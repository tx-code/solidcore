// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Observe actual ValidatePointers on public synthetic one-defect runtime graphs.
// Test-only access is necessary for absent required links. Every mutation is
// restored before context teardown, including an exception or early return.
#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmBSplineSurface.h"
#include "SmBrepData.h"
#include "SmContext.h"
#include "SmEdge.h"
#include "SmEdgeuse.h"
#include "SmFace.h"
#include "SmFaceuse.h"
#include "SmLine.h"
#include "SmLoop.h"
#include "SmLoopuse.h"
#include "SmPlane.h"
#include "SmRegion.h"
#include "SmShell.h"
#include "SmSurfaceCache.h"
#include "SmTrimSrfCache.h"
#include "SmVertex.h"
#include "SmVertexuse.h"
#include "SmGap.h"
#ifdef _WIN32
#include <windows.h>
#endif
#include <algorithm>
#include <array>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <nurbs.h>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Standard explicit-instantiation access for test-only data members. The class
// declarations, access of functions, layout and vtables remain SDK-original.
template <class Tag, typename Tag::type Member> struct MemberAccess {
    friend typename Tag::type access(Tag) { return Member; }
};
#define FIELD(Tag, Class, Member, Value)                                                           \
    struct Tag {                                                                                   \
        using type = Value Class::*;                                                               \
        friend type access(Tag);                                                                   \
    };                                                                                             \
    template struct MemberAccess<Tag, &Class::Member>;
FIELD(Infinite, SmBrep, m_pInfiniteRegion, SmRegion *)
FIELD(EdgeHead, SmBrep, m_pEdgeListHead, SmOwningTopology *)
FIELD(Owner, SmTopology, m_pListOwner, SmOwningTopology *)
FIELD(List, SmOwningTopology, m_pList, SmTopology *)
FIELD(Surface, SmFace, m_pSurface, SmSurface *)
FIELD(Primary, SmFace, m_pFU, SmFaceuse *)
FIELD(Face, SmFaceuse, m_pF, SmFace *)
FIELD(FuMate, SmFaceuse, m_pFUMate, SmFaceuse *)
FIELD(PrimaryLoop, SmLoop, m_pLU, SmLoopuse *)
FIELD(Loop, SmLoopuse, m_pL, SmLoop *)
FIELD(LuMate, SmLoopuse, m_pLUMate, SmLoopuse *)
FIELD(LoopStart, SmLoopuse, m_pEUorVU, SmTopology *)
FIELD(Curve, SmEdge, m_pCurve, SmCurve *)
FIELD(VertexUse, SmEdgeuse, m_pVU, SmVertexuse *)
FIELD(VertexGap,SmVertexuse,m_sVertexFaceGap3d,SmVertexFaceGap)
FIELD(UvCurve, SmEdgeuse, m_pUVTrimCurve, SmBSplineCurve *)
FIELD(EuOwner, SmEdgeuse, m_pSorLU, SmTopology *)
FIELD(Ccw, SmEdgeuse, m_pCCW, SmEdgeuse *)
FIELD(Cw, SmEdgeuse, m_pCW, SmEdgeuse *)
FIELD(EuOrientation, SmEdgeuse, m_eOrientation, SmOrientType)
FIELD(VuOwner, SmVertexuse, m_pSorLUorEU, SmTopology *)
FIELD(FuOrientation, SmFaceuse, m_eOrientation, SmOrientType)
FIELD(LuOrientation, SmLoopuse, m_eOrientation, SmOrientType)
FIELD(FaceTol, SmFace, m_sZoneTol3d, SmZoneTol3d)
FIELD(FaceDomain,SmFace,m_vUVDomain,SmExtent2d)
FIELD(EdgeTol, SmEdge, m_sZoneTol3d, SmZoneTol3d)
FIELD(VertexTol, SmVertex, m_sZoneTol3d, SmZoneTol3d)
FIELD(RootTol, SmBrep, m_sThisZoneTol3d, SmZoneTol3d)
FIELD(ValidateOnly,SmBrep,m_bValidateOnly,SmBoolean)
FIELD(Mark3,SmTopology,m_lMark3,ULONG)
FIELD(ListSize,SmOwningTopology,m_lListSize,ULONG)
FIELD(CacheTree,SmSurfaceCache,m_pTree,SmTree *)
FIELD(TrimSubdivisionFactor,SmSurfaceCache,m_dTrimSubdivisionFactor,double)
#undef FIELD
template <class Tag, class T> static decltype(auto) field(T *p) { return p->*access(Tag{}); }
struct EmptyEdge final : SmEdge {
    EmptyEdge() = default;
    ~EmptyEdge() override = default;
};

// Synthetic fault injection for the getter protocol, not a numerical oracle.
// The actual native getter and plain cache factory still run. Reject only the
// initial trimmed factory that would otherwise implant this face's UV boundary.
struct RejectInitialTrimPlane final : SmPlane {
    using SmPlane::SmPlane;
    SmStatus CacheMakeOrValidate(SmObjectCacheType kind, const SmAObject *object,
                                SmCacheObj *old, SmCacheObj *&created) const override {
        auto face=GetFace();
        if(kind==SM_OC_TRIMSRF && !old && face && !GetContext()->GetDoingBoolean()
            && !face->GetBrep()->m_bEditingEnabled){created=nullptr;return SM_ERR;}
        return SmPlane::CacheMakeOrValidate(kind,object,old,created);
    }
};

static void require(SmStatus s) {
    if (s != SM_SUCCESS)
        throw std::runtime_error("construction failed");
}
struct Restore {
    std::vector<std::function<void()>> actions;
    template <class T> void clear(T *&slot) {
        auto saved = slot;
        actions.emplace_back([&slot, saved] { slot = saved; });
        slot = nullptr;
    }
    template <class T> void assign(T *&slot, T *value) {
        auto saved = slot;
        actions.emplace_back([&slot, saved] { slot = saved; });
        slot = value;
    }
    ~Restore() {
        for (auto i = actions.rbegin(); i != actions.rend(); ++i)
            (*i)();
    }
};
static void quote(const wchar_t *text) {
    if (!text) {
        std::cout << "null";
        return;
    }
    std::cout << '"';
    for (; *text; ++text) {
        auto c = *text;
        if (c == L'"' || c == L'\\')
            std::cout << '\\' << static_cast<char>(c);
        else if (c == L'\n')
            std::cout << "\\n";
        else if (c == L'\r')
            std::cout << "\\r";
        else if (c == L'\t')
            std::cout << "\\t";
        else if (c >= 32 && c < 127)
            std::cout << static_cast<char>(c);
        else
            throw std::runtime_error("observer expected ASCII source labels");
    }
    std::cout << '"';
}
template <class T>
static void record(std::map<const void *, unsigned long long> &keys, T *p, unsigned index) {
    if (p)
        keys[p] = (static_cast<unsigned long long>(p->GetClassType()) << 32) | index;
}
int main(int argc, char **argv) {
    try {
        std::cout << std::setprecision(17);
        if (argc < 2 || argc > 5)
            throw std::runtime_error("defect [selection] [second-defect]");
        const std::string defect = argv[1], selection = argc > 2 ? argv[2] : "zero",
                          second = argc > 3 ? argv[3] : "none";
        const std::string mode = argc > 4 ? argv[4] : "prefix";
        const bool raw = mode == "raw_fix" || mode == "raw_check";
        const bool tolerance=mode=="tolerance_check"||mode=="tolerance_update";
        const bool box_mode=mode.starts_with("bbox_");
        const bool cache_mode=mode.starts_with("cache_");
        const bool local_mode=mode=="local_bsp";
        const bool root_mode=mode=="assert_root"||mode=="assert_root_walk";
        const bool vertex_mode=(mode.starts_with("assert_vertex")&&!mode.starts_with("assert_vertexuse"))||mode.starts_with("vertex_");
        const bool topology_mode=vertex_mode||mode.starts_with("assert_region")||mode.starts_with("assert_shell")||mode.starts_with("assert_face")||mode.starts_with("assert_loop")||mode.starts_with("assert_vertexuse")||mode.starts_with("assert_edge");
#ifdef _WIN32
        // Synthetic fault inputs must exit on a native access violation without
        // opening an interactive system dialog. Kernel calculations are unchanged.
        SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX);
#endif
        const bool side_mode=mode.starts_with("edge_side")||mode.starts_with("loop_side");
        SmContext context;
        SmBrepConstructor builder;
        auto b = builder.StartBrep(context);
        SmShell *outer = nullptr, *inner = nullptr;
        require(builder.StartShell(outer, inner));
        auto f = builder.StartFace(SM_OT_SAME);
        SmExtent2d domain(0, 0, 1, 1);
        SmSurface *source_surface;
        if(defect=="trim_failure")source_surface=new (&context) RejectInitialTrimPlane(
            SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0),SmVector2d(1,1),domain,&context);
        else source_surface=new (&context) SmPlane(SmPoint3d(0,0,defect=="surface_offset"?1.0:0.0),
            SmVector3d(1,0,0),SmVector3d(0,1,0),SmVector2d(1,1),domain,&context);
        if(defect.starts_with("bspline_")){
            const gw_SURFACE* raw=static_cast<SmBSplineSurface*>(source_surface)->GetOrCreateGwNurbPointer();
            SmSurface* replacement=new (&context) SmBSplineSurface(raw);
            delete source_surface;source_surface=replacement;
        }
        require(builder.SetFaceSurface(source_surface,domain));
        auto loop = builder.StartLoop(SM_OT_SAME);
        std::array<SmPoint3d, 3> points = {SmPoint3d(0, 0, 0), SmPoint3d(1, 0, 0),
                                           SmPoint3d(0, 1, 0)};
        std::array<SmVertex *, 3> vertices;
        for (size_t i = 0; i < 3; ++i)
            vertices[i] = builder.StartVertexOfLoop(points[i]);
        // Capture creation identities before later faces reorder radial lists.
        // Observer IDs describe arenas, never transient list positions.
        std::map<SmEdgeuse*,unsigned> allocated_uses;
        auto capture_uses=[&](SmEdge* edge,SmOrientType created_orientation){SmTArray<SmEdgeuse*> uses;edge->GetEdgeuses(uses);
            std::vector<SmEdgeuse*> fresh;
            for(ULONG i=0;i<uses.GetSize();++i)if(!allocated_uses.contains(uses[i]))fresh.push_back(uses[i]);
            // BrepConstructor creates its requested orientation first, then its
            // mate. Radial list order can start at either member of this pair.
            std::sort(fresh.begin(),fresh.end(),[=](auto a,auto b){return a->GetOrientation()==created_orientation&&b->GetOrientation()!=created_orientation;});
            for(auto use:fresh)allocated_uses[use]=unsigned(allocated_uses.size());};
        SmEdge* shared_edge=nullptr;std::array<SmEdge*,3> initial_edges{};
        for (size_t i = 0; i < 3; ++i) {
            auto created_edge=builder.StartEdge(SM_OT_SAME, nullptr, vertices[i], vertices[(i + 1) % 3]);
            initial_edges[i]=created_edge;
            if(i==0)shared_edge=created_edge;
            if (!created_edge)
                throw std::runtime_error("edge");
            if (i == 0 && defect.starts_with("reverse_")) {
                gw_CPOINT cp[5];
                double u[7] = {0, 0, .25, .5, .75, 1, 1};
                for (int j = 0; j < 5; ++j) {
                    cp[j].x = j / 4.;
                    cp[j].y = cp[j].z = 0;
                    cp[j].w = -std::numeric_limits<double>::max();
                }
                cp[2].x = -1.;
                cp[1].x = (defect == "reverse_back" ? -.5 : 1.);
                cp[3].x = (defect == "reverse_front" ? .5 : 2.);
                if (defect == "reverse_2d")
                    for (auto &p : cp)
                        p.z = std::numeric_limits<double>::max();
                if (defect == "reverse_rational")
                    for (int j = 0; j < 5; ++j) {
                        cp[j].w = j + 2.;
                        cp[j].x *= cp[j].w;
                    }
                if (defect == "reverse_flat")
                    cp[1].x = cp[0].x;
                double scale = defect == "reverse_tiny"    ? 1e-8
                               : defect == "reverse_limit" ? 5e-8
                               : defect == "reverse_above" ? 1e-7
                                                           : 1.;
                for (auto &p : cp)
                    p.x *= scale;
                require(builder.SetEdgeCurve(
                    new (&context) SmBSplineCurve(3, 4, 6, 1, cp, u, &context), FALSE));
            } else
                require(builder.SetEdgeCurve(
                    new (&context) SmLine(points[i], points[(i + 1) % 3], 3, &context), FALSE));
            require(builder.SetLimits(SmVertexNode(0.), SmVertexNode(1.)));
            require(builder.EndEdge());
            capture_uses(created_edge,SM_OT_SAME);
        }
        require(builder.EndLoop());
        if(mode.find("_hole")!=std::string::npos){
            auto add_hole=[&](std::vector<SmPoint3d> hp,bool shared){
                if(!builder.StartLoop(SM_OT_OPPOSITE))throw std::runtime_error("hole loop");
                std::vector<SmVertex*> hv;
                for(size_t i=0;i<hp.size();++i)hv.push_back(shared&&i==0?vertices[0]:builder.StartVertexOfLoop(hp[i]));
                for(size_t i=0;i<hp.size();++i){size_t j=(i+1)%hp.size();
                    if(!builder.StartEdge(SM_OT_SAME,nullptr,hv[i],hv[j]))throw std::runtime_error("hole edge");
                    require(builder.SetEdgeCurve(new (&context) SmLine(hp[i],hp[j],3,&context),FALSE));
                    require(builder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));require(builder.EndEdge());}
                require(builder.EndLoop());
            };
            if(mode.find("_outside")!=std::string::npos)add_hole({{.8,.8,0},{.8,.9,0},{.9,.9,0},{.9,.8,0}},false);
            else if(mode.find("_cross")!=std::string::npos)add_hole({{.2,.2,0},{.2,.8,0},{.8,.8,0},{.8,.2,0}},false);
            else if(mode.find("_shared")!=std::string::npos)add_hole({{0,0,0},{.2,.3,0},{.3,.2,0}},true);
            else {add_hole({{.2,.2,0},{.2,.4,0},{.4,.4,0},{.4,.2,0}},false);
                if(mode.find("_nested")!=std::string::npos)add_hole({{.25,.25,0},{.25,.35,0},{.35,.35,0},{.35,.25,0}},false);}
        }
        SmLoop *vertex_loop=nullptr;
        if(mode.find("vertexloop")!=std::string::npos)vertex_loop=builder.StartAndEndSingleVertexLoop(SmPoint3d(.2,.2,0));
        require(builder.EndFace());
        if(mode.find("_trihedral")!=std::string::npos){
            SmVertex* zv=nullptr;SmEdge* ze=nullptr;
            for(int n=0;n<2;++n){
                builder.StartFace(SM_OT_SAME);
                SmVector3d u=n==0?SmVector3d(0,0,1):SmVector3d(0,1,0),v=n==0?SmVector3d(1,0,0):SmVector3d(0,0,1);
                require(builder.SetFaceSurface(new(&context)SmPlane(points[0],u,v,SmVector2d(1,1),domain,&context),domain));
                builder.StartLoop(SM_OT_SAME);if(n==0)zv=builder.StartVertexOfLoop(SmPoint3d(0,0,1));
                SmVertex* vv[]={vertices[0],n==0?zv:vertices[2],n==0?vertices[1]:zv};
                SmPoint3d pp[]={points[0],n==0?SmPoint3d(0,0,1):points[2],n==0?points[1]:SmPoint3d(0,0,1)};
                SmEdge* existing[]={n==0?nullptr:initial_edges[2],nullptr,n==0?initial_edges[0]:ze};
                for(size_t j=0;j<3;++j){auto sense=existing[j]?SM_OT_OPPOSITE:SM_OT_SAME;
                    auto edge=builder.StartEdge(sense,existing[j],vv[j],vv[(j+1)%3]);if(!edge)throw std::runtime_error("trihedral edge");
                    if(!existing[j]){require(builder.SetEdgeCurve(new(&context)SmLine(pp[j],pp[(j+1)%3],3,&context),FALSE));require(builder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));}
                    if(n==0&&j==0)ze=edge;require(builder.EndEdge());capture_uses(edge,sense);}
                require(builder.EndLoop());require(builder.EndFace());
            }
        }
        if(mode.find("_manifold")!=std::string::npos||mode.find("_spine")!=std::string::npos){
            const int count=mode.find("_spine")!=std::string::npos?2:1;
            for(int n=0;n<count;++n){
                SmPoint3d tip=n==0?SmPoint3d(0,0,1):SmPoint3d(0,-1,0);
                builder.StartFace(SM_OT_SAME);
                require(builder.SetFaceSurface(new(&context)SmPlane(points[1],SmVector3d(-1,0,0),tip,SmVector2d(1,1),domain,&context),domain));
                builder.StartLoop(SM_OT_SAME);
                auto vertex=builder.StartVertexOfLoop(tip);
                SmVertex* vv[]={vertices[1],vertices[0],vertex};
                SmPoint3d pp[]={points[1],points[0],tip};
                for(size_t j=0;j<3;++j){
                    auto edge=builder.StartEdge(j==0?SM_OT_OPPOSITE:SM_OT_SAME,j==0?shared_edge:nullptr,vv[j],vv[(j+1)%3]);
                    if(!edge)throw std::runtime_error("radial edge");
                    if(j!=0){require(builder.SetEdgeCurve(new(&context)SmLine(pp[j],pp[(j+1)%3],3,&context),FALSE));
                        require(builder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));}
                    require(builder.EndEdge());capture_uses(edge,j==0?SM_OT_OPPOSITE:SM_OT_SAME);
                }
                require(builder.EndLoop());require(builder.EndFace());
            }
        }
        require(builder.EndShell(FALSE));
        require(builder.EndBrep());
        SmShell *target_shell=outer;
        if((mode.starts_with("assert_region")||mode.starts_with("assert_shell")||
            ((mode.starts_with("assert_vertexuse")||mode.starts_with("assert_edge")||vertex_mode)&&(mode.find("_wire")!=std::string::npos||mode.find("_shellvertex")!=std::string::npos))) &&
            (mode.find("_vertex")!=std::string::npos || mode.find("_wire")!=std::string::npos)) {
            SmVertex *vertex=nullptr;require(b->MakeShellVertexTopology(nullptr,target_shell,vertex));
            if(mode.find("_wire")!=std::string::npos) {
                SmEdge *wire=nullptr;SmVertex *end=nullptr;require(b->MakeEdgeVertexTopology(nullptr,vertex,wire,end));
                if(mode.starts_with("assert_edge")){
                    vertex->SetPoint(SmPoint3d(0,0,0));end->SetPoint(SmPoint3d(1,0,0));
                    wire->SetCurve(new(&context)SmLine(vertex->GetPoint(),end->GetPoint(),3,&context));
                    wire->SetInterval(SmExtent1d(0,1));
                }
                if(mode.find("_chain")!=std::string::npos || mode.find("_branch")!=std::string::npos) {
                    SmVertex *tip=nullptr;require(b->MakeEdgeVertexTopology(nullptr,end,wire,tip));
                }
                if(mode.find("_branch")!=std::string::npos) {
                    SmVertex *tip=nullptr;require(b->MakeEdgeVertexTopology(nullptr,vertex,wire,tip));
                }
            }
        }
        SmEdge *orphan = nullptr;
        if (defect == "edge_no_uses" || second == "edge_no_uses") {
            orphan = new (b) EmptyEdge();
            field<EdgeHead>(b)->PostInsert(orphan);
            orphan->SetCurve(new (&context) SmLine(points[0], points[1], 3, &context));
        }
        auto fu = f->GetUpwardFaceuse();
        auto mate_fu=fu->GetMate();
        auto lu = loop->GetLoopuse();
        auto target_loop=vertex_loop?vertex_loop:loop;
        auto target_lu=target_loop->GetLoopuse();
        auto target_mate_lu=target_lu->GetOtherLoopuse();
        auto first = static_cast<SmEdgeuse *>(lu->GetEUorVU());
        SmTArray<SmEdge *> edges;
        b->GetEdges(edges);
        SmTArray<SmVertex *> vs;
        b->GetVertices(vs);
        SmTArray<SmVertexuse *> vus;
        vs[0]->GetVertexuses(vus);
        auto target_vu=vus[0];
        if(mode.starts_with("assert_vertexuse")||vertex_mode) {
            if(mode.find("_vertexloop")!=std::string::npos)target_vu=target_lu->GetVertexuse();
            else if(mode.find("_wire")!=std::string::npos)target_vu=static_cast<SmEdgeuse*>(field<List>(target_shell))->GetVertexuse();
            else if(mode.find("_shellvertex")!=std::string::npos)target_vu=static_cast<SmVertexuse*>(field<List>(target_shell));
        }
        std::map<const void *, unsigned long long> keys;
        record(keys, b, 0);
        record(keys, b->GetInfiniteRegion(), 0);
        record(keys, outer, 0);
        if(target_shell!=outer)record(keys,target_shell,1);
        record(keys, f, 0);
        record(keys, fu, 0);
        record(keys, fu->GetMate(), 1);
        record(keys, loop, 0);
        keys[f->GetSurface()] = (1ULL << 63);
        record(keys, lu, 0);
        record(keys, lu->GetOtherLoopuse(), 1);
        if(mode.find("_hole")!=std::string::npos){SmTArray<SmLoop*> all;f->GetLoops(all);
            for(ULONG i=1;i<all.GetSize();++i){record(keys,all[i],i);auto use=all[i]->GetLoopuse();record(keys,use,2*i);record(keys,use->GetOtherLoopuse(),2*i+1);}}
        if(vertex_loop){record(keys,vertex_loop,1);record(keys,target_lu,2);record(keys,target_mate_lu,3);}
        if(mode.find("_manifold")!=std::string::npos||mode.find("_spine")!=std::string::npos||mode.find("_trihedral")!=std::string::npos){
            SmTArray<SmFace*> faces;b->GetFaces(faces);
            for(ULONG i=1;i<faces.GetSize();++i){auto face=faces[i];record(keys,face,i);keys[face->GetSurface()]=(1ULL<<63)|i;
                auto up=face->GetUpwardFaceuse();record(keys,up,2*i);record(keys,up->GetMate(),2*i+1);
                SmTArray<SmLoop*> loops;face->GetLoops(loops);auto l=loops[0];record(keys,l,i);
                auto use=l->GetLoopuse();record(keys,use,2*i);record(keys,use->GetOtherLoopuse(),2*i+1);}
        }
        unsigned use_index = 0, vertexuse_index = 0;
        for (ULONG i = 0; i < edges.GetSize(); ++i) {
            record(keys, edges[i], i);
            SmTArray<SmEdgeuse *> uses;
            edges[i]->GetEdgeuses(uses);
            for (ULONG j = 0; j < uses.GetSize(); ++j) {
                record(keys, uses[j], mode.find("_manifold")!=std::string::npos||mode.find("_spine")!=std::string::npos||mode.find("_trihedral")!=std::string::npos?allocated_uses.at(uses[j]):use_index++);
                record(keys, uses[j]->GetVertexuse(), mode.find("_manifold")!=std::string::npos||mode.find("_spine")!=std::string::npos||mode.find("_trihedral")!=std::string::npos?allocated_uses.at(uses[j]):vertexuse_index++ + (vertex_mode&&mode.find("_wire")!=std::string::npos&&i>=3?1:0));
            }
        }
        for (ULONG i = 0; i < vs.GetSize(); ++i){
            record(keys, vs[i], i);
            SmTArray<SmVertexuse*> uses;vs[i]->GetVertexuses(uses);
            for(ULONG j=0;j<uses.GetSize();++j)if(!keys.contains(uses[j]))record(keys,uses[j],vertexuse_index++);
        }
        if((side_mode||mode.starts_with("assert_edge")||vertex_mode) && mode.find("_uv")!=std::string::npos){SmTArray<SmEdgeuse*> uses;lu->GetEdgeuses(uses);
            for(ULONG i=0;i<uses.GetSize();++i){SmBSplineCurve *curve=nullptr;require(uses[i]->GetOrCreateUVTrimCurve(curve));}}
        if(defect=="warm_bspline"){
            // Receiver fixture: retain existing exact trims and use the same
            // analytic plane's NURB as a concrete generic B-spline receiver.
            auto old=f->GetSurface();auto raw=static_cast<SmBSplineSurface*>(old)->GetOrCreateGwNurbPointer();
            auto replacement=new(&context)SmBSplineSurface(raw);replacement->SetOwner(f);
            field<Surface>(f)=replacement;old->SetOwner(nullptr);delete old;
        }
        const SmMarkType mark_types[]={SM_MT_MARK,SM_MT_MARK2,SM_MT_MARK3,SM_MT_MARKIO,SM_MT_MARKASSERT};
        std::array<ULONG,5> before_marks{};
        for(size_t i=0;i<5;++i)before_marks[i]=context.GetCurrentMark(mark_types[i]);
        Restore restore;
        auto apply = [&](const std::string &name) {
            if (name == "none" || name == "surface_offset" || name == "trim_failure" || name == "edge_no_uses" || name=="warm_bspline" || name.starts_with("reverse_") || name.starts_with("bspline_"))
                return;
            if(name=="face_domain_outside"||name=="face_domain_line") {
                auto saved=field<FaceDomain>(f);restore.actions.emplace_back([f,saved]{field<FaceDomain>(f)=saved;});
                field<FaceDomain>(f)=name=="face_domain_line"?SmExtent2d(0,0,0,1):SmExtent2d(-.1,-.1,1.1,1.1);
            }
            else if(name=="coincident_vertices")vs[2]->SetPoint(vs[0]->GetPoint());
            else if(name=="mate_rotate") {
                auto mate=lu->GetOtherLoopuse();auto eu=static_cast<SmEdgeuse*>(mate->GetEUorVU());
                restore.assign(field<LoopStart>(mate),static_cast<SmTopology*>(eu->GetCCWEdgeuse()));
            }
            else if(name=="mate_reversed_order") {
                SmTArray<SmEdgeuse*> uses;lu->GetOtherLoopuse()->GetEdgeuses(uses);
                for(ULONG i=0;i<uses.GetSize();++i){auto next=field<Ccw>(uses[i]),last=field<Cw>(uses[i]);
                    restore.assign(field<Ccw>(uses[i]),last);restore.assign(field<Cw>(uses[i]),next);}
            }
            else if(name=="fu_empty_loops") {
                restore.clear(field<List>(fu));auto saved=field<ListSize>(fu);
                restore.actions.emplace_back([fu,saved]{field<ListSize>(fu)=saved;});field<ListSize>(fu)=0;
            }
            else if(name=="empty_shell") {
                restore.clear(field<List>(target_shell));
                auto saved=field<ListSize>(target_shell);restore.actions.emplace_back([target_shell,saved]{field<ListSize>(target_shell)=saved;});field<ListSize>(target_shell)=0;
            }
            else if(name=="region_mark3_ahead"||name=="shell_mark3_ahead") {
                SmTopology *object=name=="region_mark3_ahead"?static_cast<SmTopology*>(b->GetInfiniteRegion()):target_shell;
                auto saved=field<Mark3>(object);restore.actions.emplace_back([object,saved]{field<Mark3>(object)=saved;});
                field<Mark3>(object)=context.GetCurrentMark(SM_MT_MARK3)+1;
            }
            else if (name == "no_infinite_region")
                restore.clear(field<Infinite>(b));
            else if (name == "region_no_owner")
                restore.clear(field<Owner>(field<Infinite>(b)));
            else if (name == "shell_no_owner")
                restore.clear(field<Owner>(outer));
            else if (name == "shell_empty")
                restore.clear(field<List>(outer));
            else if (name == "face_no_surface")
                restore.clear(field<Surface>(f));
            else if (name == "face_no_primary")
                restore.clear(field<Primary>(f));
            else if (name == "fu_no_face")
                restore.clear(field<Face>(fu));
            else if (name == "fu_no_mate")
                restore.clear(field<FuMate>(fu));
            else if (name == "fu_no_shell")
                restore.clear(field<Owner>(fu));
            else if (name == "fu_no_loops")
                restore.clear(field<List>(fu));
            else if (name == "loop_no_primary")
                restore.clear(field<PrimaryLoop>(loop));
            else if (name == "lu_no_loop")
                restore.clear(field<Loop>(lu));
            else if (name == "lu_no_mate")
                restore.clear(field<LuMate>(lu));
            else if (name == "lu_no_faceuse")
                restore.clear(field<Owner>(lu));
            else if (name == "lu_no_start")
                restore.clear(field<LoopStart>(lu));
            else if (name == "edge_no_curve")
                restore.clear(field<Curve>(edges[0]));
            else if(name=="curve_no_owner"){
                auto c=edges[0]->GetCurve();auto old=c->GetOwner();restore.actions.emplace_back([c,old]{c->SetOwner(old);});c->SetOwner(nullptr);
            }
            else if(name=="edge_interval_short"||name=="edge_interval_outside")edges[0]->SetInterval(name=="edge_interval_short"?SmExtent1d(0,.5):SmExtent1d(-.1,1.1));
            else if(name=="edge_oversized_tolerance")field<EdgeTol>(edges[0])=2.;
            else if(name=="eu_wrong_orientation")field<EuOrientation>(first)=SM_OT_OPPOSITE;
            else if(name=="eu_vertex_backpointer")restore.assign(field<VuOwner>(first->GetVertexuse()),static_cast<SmTopology*>(first->GetMate()));
            else if(name=="uv_no_owner"){
                auto c=field<UvCurve>(first);if(!c)throw std::runtime_error("missing UV fixture");auto old=c->GetOwner();restore.actions.emplace_back([c,old]{c->SetOwner(old);});c->SetOwner(nullptr);
            }
            else if(name=="uv_zero_length"||name=="uv_bad_dimension"||name=="uv_interval"){
                auto old=field<UvCurve>(first);if(!old)throw std::runtime_error("missing UV fixture");
                auto c=new(&context)SmLine(SmPoint3d(0,0,0),SmPoint3d(name=="uv_zero_length"?0.:1.,0,0),name=="uv_bad_dimension"?3:2,&context);
                if(name=="uv_interval")require(c->EditParameterization(SmExtent1d(0,2)));
                c->SetOwner(first);field<UvCurve>(first)=c;restore.actions.emplace_back([first,old,c]{field<UvCurve>(first)=old;delete c;});
            }
            else if (name == "eu_no_edge")
                restore.clear(field<Owner>(first));
            else if (name == "eu_no_vertexuse")
                restore.clear(field<VertexUse>(first));
            else if (name == "eu_no_owner")
                restore.clear(field<EuOwner>(first));
            else if (name == "eu_no_ccw")
                restore.clear(field<Ccw>(first));
            else if (name == "eu_no_cw")
                restore.clear(field<Cw>(first));
            else if (name == "vu_no_vertex")
                restore.clear(field<Owner>(vus[0]));
            else if (name == "vu_no_owner")
                restore.clear(field<VuOwner>(vus[0]));
            else if (name == "vertex_no_uses")
                restore.clear(field<List>(vs[0]));
            else if (name == "loop_bad_ring") {
                auto other = first->GetCCWEdgeuse();
                restore.assign(field<Ccw>(other), other);
            } else if (name == "fu_wrong_orientation")
                field<FuOrientation>(fu) = SM_OT_OPPOSITE;
            else if (name == "mate_wrong_orientation")
                field<FuOrientation>(fu->GetMate()) = SM_OT_SAME;
            else if (name == "lu_wrong_orientation")
                field<LuOrientation>(lu) = SM_OT_OPPOSITE;
            else if (name == "mate_lu_wrong_orientation")
                field<LuOrientation>(lu->GetOtherLoopuse()) = SM_OT_OPPOSITE;
            else if (name == "face_low_tolerance")
                field<FaceTol>(f) = .000001;
            else if (name == "face_high_tolerance")
                field<FaceTol>(f) = .1;
            else if (name == "edge_low_tolerance")
                field<EdgeTol>(edges[0]) = 0.;
            else if (name == "edge_high_tolerance")
                field<EdgeTol>(edges[0]) = .1;
            else if (name == "vertex_low_tolerance")
                field<VertexTol>(vs[0]) = 0.;
            else if (name == "vertex_high_tolerance")
                field<VertexTol>(vs[0]) = .1;
            else if(name=="vertex_shift")vs[0]->SetPoint(SmPoint3d(.0003,.0004,.0005));
            else if (name == "root_zero_tolerance")
                field<RootTol>(b) = 0.;
            else if(name=="initial_validate_only")field<ValidateOnly>(b)=TRUE;
            else if(name.starts_with("gap_")){
                double factor=name=="gap_half"?.5:name=="gap_between"?.75:name=="gap_full"?1.:name=="gap_above"?2.:1000.;
                vs[0]->SetPoint(SmPoint3d(0,0,1e-5*factor));
            }
            else if(name=="locked_box"){
                require(b->LockBoundingBox(FALSE));vs[0]->SetPoint(SmPoint3d(100,100,100));
            }
            else if (name == "surface_no_owner") {
                auto surface = f->GetSurface();
                auto old = surface->GetOwner();
                restore.actions.emplace_back([surface, old] { surface->SetOwner(old); });
                surface->SetOwner(nullptr);
            } else
                throw std::runtime_error("unknown defect");
        };
        apply(defect);
        apply(second);
        SmAssertArray reports;
        SmTArray<ULONG> requested;
        requested.Add(0);
        auto level = selection.starts_with("given") ? SM_LEVEL_GIVEN
                     : selection == "one" ? SM_LEVEL_1
                     : selection == "two" ? SM_LEVEL_2
                     : selection == "heal" ? SM_LEVEL_HEAL
                                          : SM_LEVEL_0;
        if(selection=="given_face_owner"){requested.ReSet();requested.Add(4);}
        if(selection=="given_face_vertices"){requested.ReSet();requested.Add(12);requested.Add(13);}
        if(selection=="given_face_all"){requested.ReSet();for(ULONG i=0;i<40;++i)requested.Add(i);}
        auto target = selection == "no_reports" ? nullptr : &reports;
        int reversal = -1;
        SmBoolean violated=FALSE;
        double ef_trim=0,ve=0,vf=0,ef=0,vf_trim=0;
        SmEdgeuse *ef_trim_obj=nullptr,*ef_obj=nullptr,*vf_trim_obj=nullptr;
        SmVertexuse *ve_obj=nullptr,*vf_obj=nullptr;
        SmExtent3d bounds;
        SmStatus query_status=SM_SUCCESS;
        auto run_tolerance=[&]{return b->ValidateAndUpdateTolerances(mode=="tolerance_check",violated,ef_trim,ve,vf,target,level,selection=="given"?&requested:nullptr,&ef_trim_obj,&ve_obj,&vf_obj,&ef_obj,&vf_trim_obj,&ef,&vf_trim);};
        auto status =
            (raw||tolerance||box_mode||cache_mode||local_mode||root_mode||topology_mode||side_mode) ? SM_SUCCESS
                : b->ValidatePointers(target, level, selection == "given" ? &requested : nullptr,
                                      SM_NO_WALK);
        if (raw)
            reversal = N_CrvIsNotReversed(
                static_cast<SmBSplineCurve *>(edges[0]->GetCurve())->GetOrCreateGwNurbPointer(),
                mode == "raw_fix");
        if(tolerance)status=run_tolerance();
        if(box_mode){
            if(mode=="bbox_tight")query_status=b->CalculateTightBoundingBox(bounds);
            else{int flags=std::atoi(mode.c_str()+5);query_status=b->CalculateBoundingBox(bounds,(flags&1)!=0,(flags&2)!=0,(flags&4)!=0);}
            status=query_status;
        }
        if(tolerance&&selection=="repeat")status=run_tolerance();
        if (!raw && !tolerance && !box_mode && !cache_mode && !local_mode && !root_mode && !topology_mode && !side_mode && selection == "repeat")
            status = b->ValidatePointers(target, level, nullptr, SM_NO_WALK);
        bool root_valid=false;
        if(root_mode){
            const auto call=[&]{return b->AssertValid(target,level,mode=="assert_root_walk"?SM_WALK:SM_NO_WALK,
                (selection=="given"||selection=="given15")?&requested:nullptr);};
            if(selection=="given") {requested.ReSet();requested.Add(8);}
            if(selection=="given15") {level=SM_LEVEL_GIVEN;requested.ReSet();requested.Add(15);}
            root_valid=call();if(selection=="repeat")root_valid=call();
        }
        SmBoolean vertex_found=FALSE;double vertex_gap=-1.,vertex_angle=SM_BIG_DOUBLE;SmFace* vertex_face=nullptr;
        SmPoint3d vertex_point;vertex_point.SetUninitialized();SmFace* parallel_f1=nullptr;SmFace* parallel_f2=nullptr;SmPoint2d parallel_uv1,parallel_uv2;
        parallel_uv1.SetUninitialized();parallel_uv2.SetUninitialized();
        if(topology_mode){
            const auto call=[&]{auto walk=mode.ends_with("_walk")?SM_WALK:SM_NO_WALK;auto req=selection.starts_with("given")&&selection!="given_empty"?&requested:nullptr;
                if(mode.starts_with("assert_region"))return b->GetInfiniteRegion()->AssertValid(target,level,walk,req);
                if(mode.starts_with("assert_shell"))return target_shell->AssertValid(target,level,walk,req);
                if(mode.starts_with("assert_faceuse"))return (mode.find("_mate")!=std::string::npos?mate_fu:fu)->AssertValid(target,level,walk,req);
                if(mode.starts_with("assert_face"))return f->AssertValid(target,level,walk,req);
                if(mode.starts_with("assert_loopuse"))return (mode.find("_mate")!=std::string::npos?target_mate_lu:target_lu)->AssertValid(target,level,walk,req);
                if(mode.starts_with("assert_vertexuse"))return target_vu->AssertValid(target,level,walk,req);
                auto vertex=target_vu->GetVertex();
                if(mode.starts_with("assert_vertex"))return vertex->AssertValid(target,level,walk,req);
                if(mode.starts_with("vertex_pointers"))return vertex->CheckPointers();
                if(mode.starts_with("vertex_within"))return vertex->IsWithinXSectTol3dOfFaces(&vertex_gap,&vertex_face,mode.find("_force")!=std::string::npos);
                if(mode.starts_with("vertex_on_corner"))return vertex->IsOnFaceCorner(&vertex_found,&vertex_gap,&vertex_point);
                if(mode.starts_with("vertex_corner")){query_status=vertex->CalcCornerFromFaces(vertex_found,vertex_gap,vertex_face,vertex_point);return SmBoolean(query_status==SM_SUCCESS);}
                if(mode.starts_with("vertex_parallel"))return vertex->HasParallelFaces(nullptr,&vertex_angle,&vertex_gap,&parallel_f1,&parallel_f2,&parallel_uv1,&parallel_uv2);
                if(mode.starts_with("assert_edgeuse")){auto use=mode.find("_wire")!=std::string::npos?static_cast<SmEdgeuse*>(field<List>(target_shell)):first;return (mode.find("_mate")!=std::string::npos?use->GetMate():use)->AssertValid(target,level,walk,req);}
                if(mode.starts_with("assert_edge")){auto edge=mode.find("_wire")!=std::string::npos?static_cast<SmEdgeuse*>(field<List>(target_shell))->GetEdge():edges[0];return edge->AssertValid(target,level,walk,req);}
                return target_loop->AssertValid(target,level,walk,req);};
            root_valid=call();if(selection=="repeat")root_valid=call();
        }
        SmBoolean side_inside=FALSE,side_boundary=FALSE;ULONG side_index=0;
        if(side_mode){
            SmBoolean periodic=TRUE;SmBoolean* known_u=selection=="periodic_u"?&periodic:nullptr;
            SmBoolean* known_v=selection=="periodic_v"?&periodic:nullptr;
            if(mode.starts_with("loop_side"))side_inside=(mode.find("_mate")!=std::string::npos?target_mate_lu:target_lu)->IsLoopuseSideInSurface(known_u,known_v,&side_boundary,&side_index);
            else {auto use=mode.find("_last")!=std::string::npos?edges[2]->GetPrimaryEdgeuse():first;
                if(mode.find("_mate")!=std::string::npos)use=use->GetMate();
                ULONG samples=selection.starts_with("samples_")?std::strtoul(selection.c_str()+8,nullptr,10):6;
                side_inside=use->IsEdgeuseSideInSurface(samples,known_u,known_v,&side_boundary);}
        }
        SmBoolean local_found=FALSE;SmSolution local_solution;
        if(local_mode){
            const bool outside=defect=="bspline_outside",boundary=defect=="bspline_boundary";
            SmPoint3d point(outside?1.01:boundary?1.0:.2,outside?-.01:boundary?0.0:.3,.01);
            SmPoint2d guess(outside?1.0:.5,outside?0.0:.5);
            const auto solve=[&]{return f->GetSurface()->LocalPointSolve(domain,SM_SO_MINIMIZE,point,guess,local_found,local_solution);};
            status=solve();if(selection=="repeat")status=solve();
        }
        SmSurfaceCache *cache=nullptr;
        auto obtain_cache=[&]{
            b->m_bEditingEnabled=mode=="cache_editing";
            context.SetDoingBoolean(mode=="cache_boolean"||mode=="cache_boolean_tree");
            cache=smsurf_GetSurfaceCache(f->GetSurface());
            if(!cache)throw std::runtime_error("native surface cache not built");
            if(mode=="cache_reuse"||mode=="cache_rebuild"||mode=="cache_reuse_editing"||mode=="cache_rebuild_boolean"){
                field<TrimSubdivisionFactor>(cache)=37.0;
                cache->m_bFaceWasModified=mode!="cache_reuse";
                b->m_bEditingEnabled=mode=="cache_reuse_editing";
                context.SetDoingBoolean(mode=="cache_rebuild_boolean");
                cache=smsurf_GetSurfaceCache(f->GetSurface());
                if(!cache)throw std::runtime_error("native surface cache not rebuilt");
            }
            if((mode=="cache_boolean_tree"||mode=="cache_base_tree")&&!cache->GetTree())throw std::runtime_error("native delayed tree failed");
        };
        if(cache_mode){obtain_cache();if(selection=="repeat")obtain_cache();}

        for(ULONG i=0;i<edges.GetSize();++i){
            if(auto c=edges[i]->GetCurve())keys[c]=(1ULL<<62)|i;
            SmTArray<SmEdgeuse*> uses;edges[i]->GetEdgeuses(uses);
            for(ULONG j=0;j<uses.GetSize();++j)if(auto c=field<UvCurve>(uses[j]))keys[c]=(3ULL<<62)|(keys.at(uses[j])&0xffffffffULL);
        }
        std::cout << "{\"status\":" << status
                  << ",\"last_add\":" << (reports.GetLastAdd() ? "true" : "false")
                  << ",\"reports\":[";
        for (ULONG i = 0; i < reports.GetSize(); ++i) {
            if (i)
                std::cout << ',';
            auto r = reports[i];
            std::cout << "[" << r->m_lReportingType << ',' << r->m_lListIndex << ','
                      << r->m_lTestIndex << ',' << r->m_eAssertType << ',' << keys.at(r->m_pOwner)
                      << ',' << r->m_lOwnerType << ',';
            quote(r->m_pName);
            std::cout << ',';
            quote(r->m_pMessage);
            std::cout << ',' << r->m_sTol3d << ',' << r->m_dValue << ',';quote(r->m_sOwnerTypeString);
            if(mode.starts_with("assert_edge")||vertex_mode)std::cout<<",["<<r->m_sOwnerParam.x<<','<<r->m_sOwnerParam.y<<']';
            std::cout<<']';
        }
        std::cout << "]";
        if (argc == 5 && !tolerance && !box_mode) {
            auto curve = static_cast<SmBSplineCurve *>(edges[0]->GetCurve());
            std::cout << ",\"control_points\":";
            if (!curve || !curve->GetGwNurbPointer())
                std::cout << "null";
            else {
                ULONG n = 0;
                double *data = nullptr;
                require(curve->GetControlPointsPointer(n, data));
                std::cout << '[';
                for (ULONG i = 0; i < n; ++i) {
                    if (i)
                        std::cout << ',';
                    std::cout << '[' << data[4 * i] << ',' << data[4 * i + 1] << ','
                              << data[4 * i + 2] << ',' << data[4 * i + 3] << ']';
                }
                std::cout << ']';
            }
        }
        if (raw)
            std::cout << ",\"reversal_mask\":" << reversal;
        auto witness=[&](const void* p){if(!p)std::cout<<"null";else std::cout<<keys.at(p);};
        if(tolerance){
            std::cout<<",\"tolerance\":["<<(violated?"true":"false")<<','<<ef_trim<<','<<ve<<','<<vf<<','<<ef<<','<<vf_trim<<',';
            witness(ve_obj);std::cout<<',';witness(vf_obj);std::cout<<',';witness(ef_trim_obj);std::cout<<',';witness(ef_obj);std::cout<<',';witness(vf_trim_obj);std::cout<<"]";
        }
        if(tolerance||box_mode||cache_mode){
            std::cout<<",\"validate_only\":"<<(field<ValidateOnly>(b)?"true":"false")<<",\"working_tolerances\":["<<b->GetTolerance()<<','<<f->GetTolerance();
            for(ULONG i=0;i<edges.GetSize();++i)std::cout<<','<<edges[i]->GetTolerance();
            for(ULONG i=0;i<vs.GetSize();++i)std::cout<<','<<vs[i]->GetTolerance();std::cout<<']';
        }
        if(box_mode){auto a=bounds.GetMin(),z=bounds.GetMax();std::cout<<",\"box\":["<<a.x<<','<<a.y<<','<<a.z<<','<<z.x<<','<<z.y<<','<<z.z<<']';}
        if(topology_mode){
            std::cout<<",\"local_valid\":"<<(root_valid?"true":"false")<<",\"mark_advances\":[";
            for(size_t i=0;i<5;++i){if(i)std::cout<<',';std::cout<<context.GetCurrentMark(mark_types[i])-before_marks[i];}std::cout<<']';
            std::cout<<",\"current_marks\":[";
            std::vector<const SmTopology*> observed={b->GetInfiniteRegion(),target_shell,fu,mate_fu};
            for(ULONG j=0;j<vs.GetSize();++j)observed.push_back(vs[j]);
            for(ULONG j=0;j<edges.GetSize();++j)observed.push_back(edges[j]);
            for(ULONG j=0;j<vs.GetSize();++j){SmTArray<SmVertexuse*> uses;vs[j]->GetVertexuses(uses);for(ULONG n=0;n<uses.GetSize();++n)observed.push_back(uses[n]);}
            for(size_t j=0;j<observed.size();++j){if(j)std::cout<<',';std::cout<<'[';
                for(size_t i=0;i<5;++i){if(i)std::cout<<',';std::cout<<(observed[j]->GetMark(mark_types[i])==context.GetCurrentMark(mark_types[i])?"true":"false");}
                std::cout<<']';}std::cout<<']';
        }
        if(mode.starts_with("assert_edge")){
            std::vector<SmEdgeuse*> uses;
            for(ULONG i=0;i<edges.GetSize();++i){SmTArray<SmEdgeuse*> all;edges[i]->GetEdgeuses(all);for(ULONG j=0;j<all.GetSize();++j)uses.push_back(all[j]);}
            std::sort(uses.begin(),uses.end(),[&](auto a,auto b){return keys.at(a)<keys.at(b);});
            std::cout<<",\"use_state\":[";
            for(size_t i=0;i<uses.size();++i){if(i)std::cout<<',';auto use=uses[i];auto uv=field<UvCurve>(use);
                std::cout<<'['<<(keys.at(use)&0xffffffffULL)<<','<<(uv?"true":"false")<<','<<(uv&&uv->GetGwNurbPointer()?"true":"false")<<','<<(use->IsMaxEdgeFaceGapInit()?"true":"false")<<']';}
            std::cout<<']';
        }
        if(vertex_mode){
            auto vertex=target_vu->GetVertex();std::cout<<",\"vertex_point\":["<<vertex->GetPoint().x<<','<<vertex->GetPoint().y<<','<<vertex->GetPoint().z<<']';
            std::cout<<",\"vertex_state\":[";SmTArray<SmVertexuse*> uses;vertex->GetVertexuses(uses);
            for(ULONG i=0;i<uses.GetSize();++i){if(i)std::cout<<',';auto g=field<VertexGap>(uses[i]);auto uv=g.GetFaceUV();
                std::cout<<'['<<(keys.at(uses[i])&0xffffffffULL)<<','<<(g.IsInit()?"true":"false")<<','<<g.GetLength()<<','<<uv.x<<','<<uv.y<<']';}std::cout<<']';
            if(mode.starts_with("vertex_")){
                std::cout<<",\"vertex_query\":["<<query_status<<','<<(vertex_found?"true":"false")<<','<<vertex_gap<<',';
                witness(vertex_face);std::cout<<",["<<vertex_point.x<<','<<vertex_point.y<<','<<vertex_point.z<<"],"<<vertex_angle<<',';
                witness(parallel_f1);std::cout<<',';witness(parallel_f2);std::cout<<",["<<parallel_uv1.x<<','<<parallel_uv1.y<<','<<parallel_uv2.x<<','<<parallel_uv2.y<<"]]";
            }
        }
        if(side_mode)std::cout<<",\"side\":["<<(side_inside?"true":"false")<<','<<(side_boundary?"true":"false")<<','<<side_index<<']';
        if(root_mode)std::cout<<",\"root_valid\":"<<(root_valid?"true":"false");
        if(local_mode)std::cout<<",\"local\":["<<(local_found?"true":"false")<<','<<local_solution.m_vStart[0]<<','<<local_solution.m_vStart[1]<<','<<local_solution.m_vStart.m_dSolutionValue<<']';
        if(cache_mode){
            auto a=cache->GetUVDomain().GetMin(),z=cache->GetUVDomain().GetMax();
            std::cout<<",\"cache\":["<<a.x<<','<<a.y<<','<<z.x<<','<<z.y<<','
                <<(cache->m_bHaveTSurfaceCache?"true":"false")<<','
                <<(cache->m_bProcessBoundaryCurves?"true":"false")<<','
                <<(cache->m_bPointTestEnabled?"true":"false")<<','<<cache->m_bFaceContainmentDone<<','
                <<(b->m_bEditingEnabled?"true":"false")<<','<<(f->GetSurface()->GetFace()==f?"true":"false")<<']';
            std::cout<<",\"cache_role\":["<<(cache->IsKindOf(SmTrimSrfCache_TYPE)?"true":"false")<<','
                <<(field<CacheTree>(cache)?"true":"false")<<','<<field<TrimSubdivisionFactor>(cache)<<','
                <<(cache->m_bFaceWasModified?"true":"false")<<']';
            const SmPoint2d points[]={SmPoint2d(.2,.2),SmPoint2d(.8,.8),SmPoint2d(0,0),SmPoint2d(.5,0)};
            std::cout<<",\"point_tests\":[";
            for(int n=0;n<4 && field<CacheTree>(cache);++n){
                if(n)std::cout<<',';
                SmBoolean good=FALSE;SmObject *object=nullptr;
                const auto result=cache->PointTest(points[n],good,nullptr,&object);
                std::cout<<'['<<result<<','<<(good?"true":"false")<<',';witness(object);std::cout<<']';
            }
            std::cout<<']';
            std::cout<<",\"uv_trims\":[";
            for(ULONG i=0;i<edges.GetSize();++i){
                if(i)std::cout<<',';
                auto c=edges[i]->GetPrimaryEdgeuse()->GetUVTrimCurve();
                if(!c){std::cout<<"null";continue;}
                auto ivl=c->GetNaturalInterval();
                std::cout<<'['<<c->GetType()<<','<<ivl.GetMin()<<','<<ivl.GetMax()<<",[";
                for(int n=0;n<3;++n){if(n)std::cout<<',';SmPoint3d p;require(c->EvaluatePoint(ivl.GetMin()+(ivl.GetMax()-ivl.GetMin())*n/2.0,p));std::cout<<'['<<p.x<<','<<p.y<<','<<p.z<<']';}
                std::cout<<"]]";
            }
            std::cout<<']';
        }
        std::cout << "}\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
