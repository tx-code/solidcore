// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Observe actual ValidatePointers on public synthetic one-defect runtime graphs.
// Test-only access is necessary for absent required links. Every mutation is
// restored before context teardown, including an exception or early return.
#include "SmAssertArray.h"
#include "SmBrep.h"
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
#include "SmVertex.h"
#include "SmVertexuse.h"
#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
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
#undef FIELD
template <class Tag, class T> static decltype(auto) field(T *p) { return p->*access(Tag{}); }
struct EmptyEdge final : SmEdge {
    EmptyEdge() = default;
    ~EmptyEdge() override = default;
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
        if (argc < 2 || argc > 4)
            throw std::runtime_error("defect [selection] [second-defect]");
        const std::string defect = argv[1], selection = argc > 2 ? argv[2] : "zero",
                          second = argc > 3 ? argv[3] : "none";
        SmContext context;
        SmBrepConstructor builder;
        auto b = builder.StartBrep(context);
        SmShell *outer = nullptr, *inner = nullptr;
        require(builder.StartShell(outer, inner));
        auto f = builder.StartFace(SM_OT_SAME);
        SmExtent2d domain(0, 0, 1, 1);
        require(builder.SetFaceSurface(
            new (&context) SmPlane(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0),
                                   SmVector2d(1, 1), domain, &context),
            domain));
        auto loop = builder.StartLoop(SM_OT_SAME);
        std::array<SmPoint3d, 3> points = {SmPoint3d(0, 0, 0), SmPoint3d(1, 0, 0),
                                           SmPoint3d(0, 1, 0)};
        std::array<SmVertex *, 3> vertices;
        for (size_t i = 0; i < 3; ++i)
            vertices[i] = builder.StartVertexOfLoop(points[i]);
        for (size_t i = 0; i < 3; ++i) {
            if (!builder.StartEdge(SM_OT_SAME, nullptr, vertices[i], vertices[(i + 1) % 3]))
                throw std::runtime_error("edge");
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
            if (name == "none" || name == "edge_no_uses")
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
        auto status = b->ValidatePointers(target, level,
                                          selection == "given" ? &requested : nullptr, SM_NO_WALK);
        if (selection == "repeat")
            status = b->ValidatePointers(target, level, nullptr, SM_NO_WALK);
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
            std::cout << ']';
        }
        std::cout << "]}\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
