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
FIELD(EuOwner, SmEdgeuse, m_pSorLU, SmTopology *)
FIELD(Ccw, SmEdgeuse, m_pCCW, SmEdgeuse *)
FIELD(Cw, SmEdgeuse, m_pCW, SmEdgeuse *)
FIELD(VuOwner, SmVertexuse, m_pSorLUorEU, SmTopology *)
FIELD(FuOrientation, SmFaceuse, m_eOrientation, SmOrientType)
FIELD(LuOrientation, SmLoopuse, m_eOrientation, SmOrientType)
FIELD(FaceTol, SmFace, m_sZoneTol3d, SmZoneTol3d)
FIELD(EdgeTol, SmEdge, m_sZoneTol3d, SmZoneTol3d)
FIELD(VertexTol, SmVertex, m_sZoneTol3d, SmZoneTol3d)
FIELD(RootTol, SmBrep, m_sThisZoneTol3d, SmZoneTol3d)
FIELD(ValidateOnly,SmBrep,m_bValidateOnly,SmBoolean)
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
        for (size_t i = 0; i < 3; ++i) {
            if (!builder.StartEdge(SM_OT_SAME, nullptr, vertices[i], vertices[(i + 1) % 3]))
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
        }
        require(builder.EndLoop());
        require(builder.EndFace());
        require(builder.EndShell(FALSE));
        require(builder.EndBrep());
        SmEdge *orphan = nullptr;
        if (defect == "edge_no_uses" || second == "edge_no_uses") {
            orphan = new (b) EmptyEdge();
            field<EdgeHead>(b)->PostInsert(orphan);
            orphan->SetCurve(new (&context) SmLine(points[0], points[1], 3, &context));
        }
        auto fu = f->GetUpwardFaceuse();
        auto lu = loop->GetLoopuse();
        auto first = static_cast<SmEdgeuse *>(lu->GetEUorVU());
        SmTArray<SmEdge *> edges;
        b->GetEdges(edges);
        SmTArray<SmVertex *> vs;
        b->GetVertices(vs);
        SmTArray<SmVertexuse *> vus;
        vs[0]->GetVertexuses(vus);
        std::map<const void *, unsigned long long> keys;
        record(keys, b, 0);
        record(keys, b->GetInfiniteRegion(), 0);
        record(keys, outer, 0);
        record(keys, f, 0);
        record(keys, fu, 0);
        record(keys, fu->GetMate(), 1);
        record(keys, loop, 0);
        keys[f->GetSurface()] = (1ULL << 63);
        record(keys, lu, 0);
        record(keys, lu->GetOtherLoopuse(), 1);
        unsigned use_index = 0, vertexuse_index = 0;
        for (ULONG i = 0; i < edges.GetSize(); ++i) {
            record(keys, edges[i], i);
            SmTArray<SmEdgeuse *> uses;
            edges[i]->GetEdgeuses(uses);
            for (ULONG j = 0; j < uses.GetSize(); ++j) {
                record(keys, uses[j], use_index++);
                record(keys, uses[j]->GetVertexuse(), vertexuse_index++);
            }
        }
        for (ULONG i = 0; i < vs.GetSize(); ++i)
            record(keys, vs[i], i);
        Restore restore;
        auto apply = [&](const std::string &name) {
            if (name == "none" || name == "surface_offset" || name == "trim_failure" || name == "edge_no_uses" || name.starts_with("reverse_") || name.starts_with("bspline_"))
                return;
            if (name == "no_infinite_region")
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
        auto level = selection == "given" ? SM_LEVEL_GIVEN
                     : selection == "two" ? SM_LEVEL_2
                                          : SM_LEVEL_0;
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
            (raw||tolerance||box_mode||cache_mode||local_mode) ? SM_SUCCESS
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
        if (!raw && !tolerance && !box_mode && !cache_mode && !local_mode && selection == "repeat")
            status = b->ValidatePointers(target, level, nullptr, SM_NO_WALK);
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
            std::cout << ',' << r->m_sTol3d << ',' << r->m_dValue << ']';
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
