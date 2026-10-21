// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Host observer only. All collection and mark operations are the actual SDK's.
#pragma once
#include "SmTopologyTraverser.h"
struct SubFixture {
    SmBrep* b;SmRegion* region;SmShell* shell;SmFace* face;SmLoop* loop;
    SmEdge* edge;SmVertex* vertex;SmFaceuse* fu;SmLoopuse* lu;SmEdgeuse* eu;SmVertexuse* vu;
};
struct SubObserved {SmStatus status=SM_SUCCESS;std::string output,advances,marks;};
using SubKeys=std::map<const void*,unsigned long long>;
static const SmTopology* sub_topology_pointer(const void* p,unsigned long long key,const SmBrep* b){
    if(p==b)return b;
    // Typed casts let the compiler perform the correct base adjustment.
    switch(key>>32){
        case SmRegion_TYPE:return static_cast<const SmRegion*>(p);
        case SmShell_TYPE:return static_cast<const SmShell*>(p);
        case SmFace_TYPE:return static_cast<const SmFace*>(p);
        case SmLoop_TYPE:return static_cast<const SmLoop*>(p);
        case SmEdge_TYPE:return static_cast<const SmEdge*>(p);
        case SmVertex_TYPE:return static_cast<const SmVertex*>(p);
        case SmFaceuse_TYPE:return static_cast<const SmFaceuse*>(p);
        case SmLoopuse_TYPE:return static_cast<const SmLoopuse*>(p);
        case SmEdgeuse_TYPE:return static_cast<const SmEdgeuse*>(p);
        case SmVertexuse_TYPE:return static_cast<const SmVertexuse*>(p);
        default:return nullptr;
    }
}
static SmTopology* sub_object(const SubFixture& f,std::string kind){
    if(kind=="brep")return f.b;if(kind=="region")return f.region;if(kind=="shell")return f.shell;
    if(kind=="face")return f.face;if(kind=="loop")return f.loop;if(kind=="edge")return f.edge;if(kind=="vertex")return f.vertex;
    if(kind=="faceuse")return f.fu;if(kind=="loopuse")return f.lu;if(kind=="edgeuse")return f.eu;if(kind=="vertexuse")return f.vu;
    throw std::runtime_error("invalid subtopology receiver");
}
static SubObserved observe_subtopology(SmContext& context,const SubKeys& keys,const SubFixture& f,const std::string& mode,std::string cfg){
    std::replace(cfg.begin(),cfg.end(),',',' ');std::istringstream in(cfg);int mask,reset,target,locked,repeats,seed,premark;
    if(!(in>>mask>>reset>>target>>locked>>repeats>>seed>>premark)||mask<0||mask>63||repeats<1||repeats>2||seed<0||seed>2)throw std::runtime_error("invalid subtopology configuration");
    auto kind=mode.substr(4);kind=kind.substr(0,kind.find('_'));auto object=sub_object(f,kind);
    auto mark=static_cast<SmMarkType>(target);std::unique_ptr<SmNewMarkAndLock> target_lock;
    if(locked&&mark!=SM_MT_NOMARK)target_lock=std::make_unique<SmNewMarkAndLock>(&context,target);
    std::vector<std::pair<unsigned long long,const SmTopology*>> pool;
    for(auto [p,key]:keys)if(auto topo=sub_topology_pointer(p,key,f.b))pool.emplace_back(key,topo);
    std::sort(pool.begin(),pool.end(),[](auto a,auto b){return a.first<b.first;});
    if(mark!=SM_MT_NOMARK)for(auto [key,p]:pool)if(premark==2||(premark==1&&p==object))const_cast<SmTopology*>(p)->Mark(mark);
    SmTArray<SmRegion*> regions;SmTArray<SmShell*> shells;SmTArray<SmFace*> faces;
    SmTArray<SmLoop*> loops;SmTArray<SmEdge*> edges;SmTArray<SmVertex*> vertices;
    for(int i=0;i<seed;++i){if(mask&1)regions.Add(f.region);if(mask&2)shells.Add(f.shell);if(mask&4)faces.Add(f.face);if(mask&8)loops.Add(f.loop);if(mask&16)edges.Add(f.edge);if(mask&32)vertices.Add(f.vertex);}
    const SmMarkType types[]={SM_MT_MARK,SM_MT_MARK2,SM_MT_MARK3,SM_MT_MARKIO,SM_MT_MARKASSERT};
    std::array<ULONG,5> before{};for(size_t i=0;i<5;++i)before[i]=context.GetCurrentMark(types[i]);
    SubObserved result;
    for(int i=0;i<repeats;++i)result.status=SmTopologyTraverser::GetSubTopology(*object,mask&1?&regions:nullptr,mask&2?&shells:nullptr,mask&4?&faces:nullptr,mask&8?&loops:nullptr,mask&16?&edges:nullptr,mask&32?&vertices:nullptr,reset,mark);
    std::ostringstream output;output<<'[';
    auto array=[&](const auto& values,bool requested){if(!requested){output<<"null";return;}output<<'[';for(ULONG i=0;i<values.GetSize();++i){if(i)output<<',';output<<keys.at(values[i]);}output<<']';};
    array(regions,mask&1);output<<',';array(shells,mask&2);output<<',';array(faces,mask&4);output<<',';array(loops,mask&8);output<<',';array(edges,mask&16);output<<',';array(vertices,mask&32);output<<']';result.output=output.str();
    std::ostringstream advances;advances<<'[';for(size_t i=0;i<5;++i){if(i)advances<<',';advances<<context.GetCurrentMark(types[i])-before[i];}advances<<']';result.advances=advances.str();
    std::ostringstream marks;marks<<'[';for(size_t j=0;j<pool.size();++j){if(j)marks<<',';auto [key,p]=pool[j];marks<<'['<<key<<",[";
        for(size_t i=0;i<5;++i){if(i)marks<<',';marks<<(p->IsMarked(types[i])?"true":"false");}marks<<"]]";}marks<<']';result.marks=marks.str();
    return result;
}
