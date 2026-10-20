// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Host-only observer of the maintained fork constructor (d8ed45a). Synthetic
// construction evidence is separate from native acceptance and lazy validation.
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
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

static void require(SmStatus status) {
    if (status != SM_SUCCESS)
        throw std::runtime_error("constructor operation failed");
}
template <class T> static std::vector<T *> list(const SmTArray<T *> &values) {
    std::vector<T *> result;
    for (ULONG i = 0; i < values.GetSize(); ++i)
        result.push_back(values[i]);
    return result;
}
template <class T> static int index(const std::vector<T *> &values, const T *value) {
    if (!value)
        return -1;
    auto it = std::find(values.begin(), values.end(), value);
    if (it == values.end())
        throw std::runtime_error("observer: missing identity");
    return static_cast<int>(it - values.begin());
}
template <class T, class F> static void array(const std::vector<T> &values, F emit) {
    std::cout << '[';
    bool first = true;
    for (const auto &value : values) {
        if (!first)
            std::cout << ',';
        first = false;
        emit(value);
    }
    std::cout << ']';
}
static int orient(SmOrientType value) {
    return value == SM_OT_SAME ? 1 : value == SM_OT_OPPOSITE ? -1 : 0;
}
static void point(const SmPoint3d &p) {
    std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}
static double dot(const SmVector3d &a, const SmVector3d &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static void quote(const wchar_t *p) {
    if (!p) {
        std::cout << "null";
        return;
    }
    std::cout << '"';
    for (; *p; ++p) {
        if (*p == L'"' || *p == L'\\')
            std::cout << '\\' << static_cast<char>(*p);
        else if (*p == L'\n')
            std::cout << "\\n";
        else if (*p == L'\r')
            std::cout << "\\r";
        else if (*p == L'\t')
            std::cout << "\\t";
        else if (*p >= 32 && *p < 127)
            std::cout << static_cast<char>(*p);
        else
            throw std::runtime_error("non-ASCII fixed label");
    }
    std::cout << '"';
}

int main(int argc, char **argv) {
    try {
        if (argc != 9 && argc != 10)
            throw std::runtime_error("shape walk outside vertices uv point-limits scale offset");
        std::cout << std::setprecision(17);
        const std::string shape = argv[1];
        const auto walk = std::atoi(argv[2]) == 1 ? SM_OT_SAME : SM_OT_OPPOSITE;
        const auto outside = std::atoi(argv[3]) == 1 ? SM_OT_SAME : SM_OT_OPPOSITE;
        const bool explicit_vertices = std::atoi(argv[4]) != 0;
        const bool uv = std::atoi(argv[5]) != 0;
        const bool point_limits = std::atoi(argv[6]) != 0;
        const double scale = std::stod(argv[7]), offset = std::stod(argv[8]);
        std::vector<SmPoint3d> points = {{0, 0, 0},     {2, 0, 0},     {2, 3, 0},     {0, 3, 0},
                                         {0, 0, 4},     {2, 0, 4},     {2, 3, 4},     {0, 3, 4},
                                         {0.5, 0.5, 0}, {1.5, 0.5, 0}, {1.5, 2.5, 0}, {0.5, 2.5, 0},
                                         {2, -3, 0},    {0, -3, 0}};
        for (auto &p : points)
            p = SmPoint3d(p.x * scale + offset, p.y * scale + offset, p.z * scale + offset);
        std::vector<std::vector<std::vector<int>>> faces;
        if (shape == "plane" || shape == "vertex" || shape == "lift")
            faces = {{{0, 1, 2, 3}}};
        else if (shape == "hole")
            faces = {{{0, 1, 2, 3}, {8, 9, 10, 11}}};
        else if (shape == "two")
            faces = {{{0, 1, 2, 3}}, {{2, 1, 5, 6}}};
        else if (shape == "radial")
            faces = {{{0, 1, 2, 3}}, {{1, 0, 4, 5}}, {{1, 0, 13, 12}}};
        else if (shape == "solid" || shape == "surface")
            faces = {{{0, 3, 2, 1}}, {{4, 5, 6, 7}}, {{0, 1, 5, 4}},
                     {{1, 2, 6, 5}}, {{2, 3, 7, 6}}, {{3, 0, 4, 7}}};
        else
            throw std::runtime_error("unknown shape");
        SmContext context;
        SmBrepConstructor builder;
        SmBrep *brep = builder.StartBrep(context);
        if (!brep)
            throw std::runtime_error("StartBrep failed");
        // Context owns unfinished geometry too. No object escapes on exceptions.
        brep->SetTolerance(1.e-5);
        SmRegion *region = nullptr;
        if (shape == "solid") {
            region = builder.StartRegion(TRUE, nullptr);
            region->SetIsVoid(FALSE);
        }
        SmShell *outer = nullptr, *inner = nullptr;
        require(builder.StartShell(outer, inner));
        std::map<int, SmVertex *> vertices;
        if (explicit_vertices && shape != "vertex") {
            for (const auto &face : faces)
                for (const auto &loop : face)
                    for (int i : loop)
                        if (!vertices.count(i))
                            vertices[i] = builder.StartVertexOfLoop(points.at(i));
        }
        struct Defined {
            SmEdge *edge;
            int start;
            int end;
        };
        std::map<std::pair<int, int>, Defined> defined;
        for (const auto &face : faces) {
            auto f = builder.StartFace(outside);
            if (!f)
                throw std::runtime_error("StartFace failed");
            const auto &polygon = face.front();
            const auto p = points.at(polygon.at(0));
            const auto u = points.at(polygon.at(1)) - p;
            const auto v = points.at(polygon.at(3)) - p;
            SmExtent2d domain(0, 0, 1, 1);
            require(builder.SetFaceSurface(
                new (&context)
                    SmPlane(p, u, v, SmVector2d(u.Length(), v.Length()), domain, &context),
                domain));
            if (shape == "vertex") {
                if (!builder.StartAndEndSingleVertexLoop(p))
                    throw std::runtime_error("vertex loop");
            } else
                for (size_t l = 0; l < face.size(); ++l) {
                    auto loop_walk =
                        l == 0 ? walk : (walk == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME);
                    if (!builder.StartLoop(loop_walk))
                        throw std::runtime_error("StartLoop failed");
                    const auto &loop = face.at(l);
                    for (size_t i = 0; i < loop.size(); ++i) {
                        int a = loop.at(i), b = loop.at((i + 1) % loop.size());
                        auto key = std::minmax(a, b);
                        auto it = defined.find(key);
                        const bool fresh = it == defined.end();
                        SmOrientType sense =
                            fresh || it->second.start == a ? SM_OT_SAME : SM_OT_OPPOSITE;
                        auto e = builder.StartEdge(sense, fresh ? nullptr : it->second.edge,
                                                   explicit_vertices ? vertices.at(a) : nullptr,
                                                   explicit_vertices ? vertices.at(b) : nullptr);
                        if (!e)
                            throw std::runtime_error("StartEdge failed");
                        if (fresh) {
                            if (shape == "lift") {
                                auto project = [&](const SmPoint3d &q) {
                                    const auto d = q - p;
                                    return SmPoint3d(dot(d, u) / dot(u, u), dot(d, v) / dot(v, v),
                                                     0);
                                };
                                require(builder.SetEdgeCurve(
                                    new (&context) SmLine(project(points.at(a)),
                                                          project(points.at(b)), 2, &context),
                                    TRUE));
                            } else
                                require(builder.SetEdgeCurve(
                                    new (&context) SmLine(points.at(a), points.at(b), 3, &context),
                                    FALSE));
                            if (point_limits)
                                require(builder.SetLimits(SmVertexNode(points.at(a)),
                                                          SmVertexNode(points.at(b))));
                            else
                                require(builder.SetLimits(SmVertexNode(0.), SmVertexNode(1.)));
                            defined.emplace(key, Defined{e, a, b});
                        }
                        if (uv && shape != "lift") {
                            // Local storage on the current use intentionally follows
                            // SetEdgeCurve; canonical SAME-side trim installation is a
                            // separate operation.
                            const auto definition = defined.at(key);
                            auto project = [&](const SmPoint3d &q) {
                                const auto d = q - p;
                                return SmPoint3d(dot(d, u) / dot(u, u), dot(d, v) / dot(v, v), 0);
                            };
                            auto ua = project(points.at(definition.start)),
                                 ub = project(points.at(definition.end));
                            require(builder.SetEdgeCurve(new (&context) SmLine(ua, ub, 2, &context),
                                                         TRUE));
                        }
                        require(builder.EndEdge());
                    }
                    require(builder.EndLoop());
                }
            require(builder.EndFace());
        }
        require(builder.EndShell(argc == 10 && std::string(argv[9]) == "stitch"));
        if (region)
            require(builder.EndRegion());
        require(builder.EndBrep());
        SmAssertArray validation;
        SmStatus validation_status = SM_SUCCESS;
        if (argc == 10)
            validation_status =
                brep->ValidatePointers(&validation, SM_LEVEL_2, nullptr, SM_NO_WALK);
        SmTArray<SmRegion *> rr;
        brep->GetRegions(rr);
        const auto regions = list(rr);
        SmTArray<SmShell *> ss;
        brep->GetShells(ss);
        const auto shells = list(ss);
        SmTArray<SmFace *> ff;
        brep->GetFaces(ff);
        const auto fs = list(ff);
        SmTArray<SmEdge *> ee;
        brep->GetEdges(ee);
        const auto edges = list(ee);
        SmTArray<SmVertex *> vv;
        brep->GetVertices(vv);
        const auto vs = list(vv);
        std::vector<SmLoopuse *> lus;
        for (auto f : fs) {
            SmTArray<SmLoop *> ll;
            f->GetLoops(ll);
            for (auto loop : list(ll)) {
                auto lu = loop->GetLoopuse();
                lus.push_back(lu);
                lus.push_back(lu->GetOtherLoopuse());
            }
        }
        std::vector<SmEdgeuse *> eus;
        for (auto e : edges) {
            SmTArray<SmEdgeuse *> uu;
            e->GetEdgeuses(uu);
            for (auto eu : list(uu))
                eus.push_back(eu);
        }
        std::cout << "{\"manifold\":" << (builder.IsManifold() ? "true" : "false")
                  << ",\"regions\":";
        array(regions, [&](auto r) {
            std::cout << '[' << r->IsVoid() << ',';
            SmTArray<SmShell *> s;
            r->GetShells(s);
            array(list(s), [&](auto x) { std::cout << index(shells, x); });
            std::cout << ']';
        });
        std::cout << ",\"shells\":";
        array(shells, [&](auto s) {
            SmTArray<SmFaceuse *> fu;
            s->GetFaceuses(fu);
            array(list(fu), [&](auto x) {
                std::cout << '[' << index(fs, x->GetFace()) << ',' << orient(x->GetOrientation())
                          << ']';
            });
        });
        std::cout << ",\"faces\":";
        array(fs, [&](auto f) {
            auto fu = f->GetUpwardFaceuse();
            auto d = f->GetUVDomain();
            std::cout << '[' << f->GetTolerance() << ',' << index(shells, fu->GetShell()) << ','
                      << index(shells, fu->GetMate()->GetShell()) << ','
                      << (f->GetSurface()->GetOwner() == f ? "true" : "false") << ",["
                      << d.GetUMin() << ',' << d.GetVMin() << ',' << d.GetUMax() << ','
                      << d.GetVMax() << "],";
            SmPoint3d p;
            require(f->GetSurface()->EvaluatePoint(SmPoint2d(.25, .75), p));
            point(p);
            std::cout << ']';
        });
        std::cout << ",\"loops\":";
        array(lus, [&](auto lu) {
            std::cout << '[' << index(fs, lu->GetFaceuse()->GetFace()) << ','
                      << orient(lu->GetFaceuse()->GetOrientation()) << ','
                      << orient(lu->GetOrientation()) << ',' << index(lus, lu->GetOtherLoopuse())
                      << ',';
            if (lu->IsVertexLoopuse())
                std::cout << index(vs, lu->GetVertexuse()->GetVertex());
            else
                std::cout << -1;
            std::cout << ',';
            SmTArray<SmEdgeuse *> es;
            if (lu->IsEdgeLoopuse())
                lu->GetEdgeuses(es);
            array(list(es), [&](auto x) { std::cout << index(eus, x); });
            std::cout << ']';
        });
        std::cout << ",\"edges\":";
        array(edges, [&](auto e) {
            auto c = e->GetCurve();
            auto d = e->GetInterval();
            std::cout << '[' << e->GetTolerance() << ',' << d.GetMin() << ',' << d.GetMax() << ','
                      << c->GetDim() << ',' << (c->GetOwner() == e ? "true" : "false") << ",[";
            for (int i = 0; i < 3; ++i) {
                if (i)
                    std::cout << ',';
                SmPoint3d p;
                require(c->EvaluatePoint(d.GetMin() + (d.GetMax() - d.GetMin()) * i / 2., p));
                point(p);
            }
            std::cout << "]]";
        });
        std::cout << ",\"uses\":";
        array(eus, [&](auto u) {
            auto uv = u->GetUVTrimCurvePointer();
            std::cout << '[' << index(edges, u->GetEdge()) << ',' << index(lus, u->GetLoopuse())
                      << ',' << orient(u->GetOrientation()) << ','
                      << index(vs, u->GetVertexuse()->GetVertex()) << ','
                      << index(eus, u->GetMate()) << ',' << index(eus, u->GetRadial()) << ','
                      << index(eus, u->GetCWEdgeuse()) << ',' << index(eus, u->GetCCWEdgeuse())
                      << ',';
            if (!uv)
                std::cout << "null";
            else {
                auto d = uv->GetNaturalInterval();
                std::cout << '[' << uv->GetDim() << ',' << (uv->GetOwner() == u ? "true" : "false")
                          << ',' << d.GetMin() << ',' << d.GetMax() << ",[";
                for (int i = 0; i < 3; ++i) {
                    if (i)
                        std::cout << ',';
                    SmPoint3d p;
                    require(uv->EvaluatePoint(d.GetMin() + (d.GetMax() - d.GetMin()) * i / 2., p));
                    point(p);
                }
                std::cout << "]]";
            }
            std::cout << ']';
        });
        std::cout << ",\"vertices\":";
        array(vs, [&](auto v) {
            std::cout << '[';
            point(v->GetPoint());
            std::cout << ',' << v->GetTolerance() << ',';
            SmTArray<SmVertexuse *> uses;
            v->GetVertexuses(uses);
            array(list(uses), [&](auto u) {
                if (u->GetVertexuseType() == SmEdgeuse_TYPE)
                    std::cout << index(eus, u->GetEdgeuse());
                else
                    std::cout << -1 - index(lus, u->GetLoopuse());
            });
            std::cout << ']';
        });
        if (argc == 10) {
            std::cout << ",\"pointer_validation\":{\"status\":" << validation_status
                      << ",\"reports\":[";
            for (ULONG i = 0; i < validation.GetSize(); ++i) {
                if (i)
                    std::cout << ',';
                auto r = validation[i];
                long long owner = -1;
                switch (r->m_lOwnerType) {
                case SmBrep_TYPE:
                    owner = 0;
                    break;
                case SmRegion_TYPE:
                    owner = index(regions, static_cast<const SmRegion *>(r->m_pOwner));
                    break;
                case SmShell_TYPE:
                    owner = index(shells, static_cast<const SmShell *>(r->m_pOwner));
                    break;
                case SmFace_TYPE:
                    owner = index(fs, static_cast<const SmFace *>(r->m_pOwner));
                    break;
                case SmFaceuse_TYPE: {
                    auto fu = static_cast<const SmFaceuse *>(r->m_pOwner);
                    owner = 2 * index(fs, fu->GetFace()) +
                            (fu == fu->GetFace()->GetUpwardFaceuse() ? 0 : 1);
                    break;
                }
                case SmLoop_TYPE:
                    owner = index(lus, static_cast<const SmLoop *>(r->m_pOwner)->GetLoopuse()) / 2;
                    break;
                case SmLoopuse_TYPE:
                    owner = index(lus, static_cast<const SmLoopuse *>(r->m_pOwner));
                    break;
                case SmEdge_TYPE:
                    owner = index(edges, static_cast<const SmEdge *>(r->m_pOwner));
                    break;
                case SmEdgeuse_TYPE:
                    owner = index(eus, static_cast<const SmEdgeuse *>(r->m_pOwner));
                    break;
                case SmVertex_TYPE:
                    owner = index(vs, static_cast<const SmVertex *>(r->m_pOwner));
                    break;
                case SmVertexuse_TYPE: {
                    auto vu = static_cast<const SmVertexuse *>(r->m_pOwner);
                    owner = vu->GetVertexuseType() == SmEdgeuse_TYPE
                                ? index(eus, vu->GetEdgeuse())
                                : -1 - index(lus, vu->GetLoopuse());
                    break;
                }
                case SmSurface_TYPE:
                    for (size_t j = 0; j < fs.size(); ++j)
                        if (fs[j]->GetSurface() == r->m_pOwner)
                            owner = static_cast<long long>(j);
                    break;
                default:
                    throw std::runtime_error("unmapped validation object");
                }
                std::cout << '[' << r->m_lReportingType << ',' << r->m_lListIndex << ','
                          << r->m_lTestIndex << ',' << r->m_lOwnerType << ',' << owner << ',';
                quote(r->m_pName);
                std::cout << ',';
                quote(r->m_pMessage);
                std::cout << ',' << r->m_sTol3d << ',' << r->m_dValue << ']';
            }
            std::cout << "]}";
        }
        std::cout << "}\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
