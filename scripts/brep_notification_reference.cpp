// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Direct SDK observer on a public synthetic graph, not an import qualification.
#include "SmBrep.h"
#include "SmBrepData.h"
#include "SmBrepCache.h"
#include "SmCacheMgr.h"
#include "SmContext.h"
#include "SmAttribute.h"
#include "SmFace.h"
#include "SmEdge.h"
#include "SmVertex.h"
#include "SmLine.h"
#include "SmPlane.h"
#include "SmTree.h"
#include <array>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <stdexcept>

template<class Tag,typename Tag::type Member>struct Access {friend typename Tag::type field(Tag){return Member;}};
#define FIELD(Tag, Member) struct Tag{using type=ULONG SmTopology::*;friend type field(Tag);}; template struct Access<Tag,&SmTopology::Member>;
FIELD(Mark1,m_lMark) FIELD(Mark2,m_lMark2) FIELD(Mark3,m_lMark3)
#undef FIELD
template<class Tag>static ULONG &mark(SmTopology *p){return p->*field(Tag{});}
static void require(SmStatus s){if(s!=SM_SUCCESS)throw std::runtime_error("SDK operation failed");}
using Key=unsigned long long;
using Row=std::array<Key,6>;
struct Capture final:SmNotifyCallback {
    Key tag;std::vector<Row>&rows;const std::map<const void*,Key>&keys;
    Capture(Key t,std::vector<Row>&r,const std::map<const void*,Key>&k):tag(t),rows(r),keys(k){}
    Key id(SmObject*p)const{return p?keys.at(p):0;}
    SmStatus Execute(SmNotifyOperation n,SmObject*c,SmObject*a,SmObject*b,SmObject*d)override{
        rows.push_back({tag,static_cast<Key>(n),id(c),id(a),id(b),id(d)});return SM_ERR;
    }
};
int main(int argc,char**argv){try{
    if(argc!=4)throw std::runtime_error("event cache[0/1] repeat[0/1]");
    std::cout<<std::setprecision(17);
    std::string mode=argv[1];bool cached=std::string(argv[2])=="1",repeat=std::string(argv[3])=="1";
    SmContext context;SmBrepConstructor builder;auto b=builder.StartBrep(context);
    SmShell *outer=nullptr,*inner=nullptr;require(builder.StartShell(outer,inner));
    auto face=builder.StartFace(SM_OT_SAME);SmExtent2d domain(0,0,1,1);
    require(builder.SetFaceSurface(new(&context)SmPlane(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0),SmVector2d(1,1),domain,&context),domain));
    builder.StartLoop(SM_OT_SAME);
    SmPoint3d points[]={SmPoint3d(0,0,0),SmPoint3d(1,0,0),SmPoint3d(0,1,0)};
    std::array<SmVertex*,3>v;std::array<SmEdge*,3>e;
    for(int i=0;i<3;++i)v[i]=builder.StartVertexOfLoop(points[i]);
    for(int i=0;i<3;++i){e[i]=builder.StartEdge(SM_OT_SAME,nullptr,v[i],v[(i+1)%3]);require(builder.SetEdgeCurve(new(&context)SmLine(points[i],points[(i+1)%3],3,&context),FALSE));require(builder.SetLimits(0.0,1.0));require(builder.EndEdge());}
    require(builder.EndLoop());require(builder.EndFace());require(builder.EndShell(FALSE));require(builder.EndBrep());
    std::map<const void*,Key>keys;auto bind=[&](SmObject*p,unsigned i){keys[p]=(static_cast<Key>(p->GetType())<<32)|i;};
    bind(b,0);bind(face,0);for(int i=0;i<3;++i){bind(v[i],i);bind(e[i],i);}
    std::array<SmTopology*,4>objects={b,v[0],v[1],v[2]};
    for(unsigned i=0;i<4;++i){objects[i]->SetAllFlags(i?1u<<(i-1):16);mark<Mark1>(objects[i])=3+2*i;mark<Mark2>(objects[i])=3+2*i;mark<Mark3>(objects[i])=3+2*i;objects[i]->AddAttribute(new(context)SmLongAttribute(90001,21+10*i));}
    if(cached&&!SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,b))throw std::runtime_error("cache creation failed");
    SmNotifyOperation op=SM_NO_UNKNOWN;SmObject*a=nullptr,*c=nullptr,*d=nullptr;
    if(mode=="pre_edit"){op=SM_NO_PRE_EDIT;a=b;}
    else if(mode=="destruction"){op=SM_NO_DESTRUCTION;a=b;}
    else if(mode=="post_edit"){op=SM_NO_POST_EDIT;a=b;}
    else if(mode=="add_v"){op=SM_NO_ADD_TO_BREP;a=v[0];c=b;}
    else if(mode=="remove_v"){op=SM_NO_RM_FROM_BREP;a=v[0];c=b;}
    else if(mode=="add_e"){op=SM_NO_ADD_TO_BREP;a=e[0];c=b;}
    else if(mode=="remove_e"){op=SM_NO_RM_FROM_BREP;a=e[0];c=b;}
    else if(mode=="add_f"){op=SM_NO_ADD_TO_BREP;a=face;c=b;}
    else if(mode=="remove_f"){op=SM_NO_RM_FROM_BREP;a=face;c=b;}
    else if(mode=="split"||mode=="split_alias"){op=SM_NO_SPLIT_IN_BREP;a=v[0];c=mode=="split_alias"?v[0]:v[1];d=v[2];}
    else if(mode=="merge"){op=SM_NO_MERGE_IN_BREP;a=v[0];c=v[1];d=b;}
    else if(mode=="copy"){op=SM_NO_COPY;a=v[0];}
    else if(mode=="construct"){op=SM_NO_CONSTRUCTION;a=b;c=v[0];}
    else if(mode=="coincident"){op=SM_NO_COINCIDENT;a=v[0];c=v[1];d=b;}
    else if(mode!="unknown")throw std::runtime_error("unknown event");
    std::vector<Row>events;Capture user(0,events,keys),system(1,events,keys);
    context.SetUserNotifyCallback(&user);context.SetSysNotifyCallback(&system);
    b->Notify(op,a,c,d);if(repeat)b->Notify(op,a,c,d);
    context.SetUserNotifyCallback(nullptr);context.SetSysNotifyCallback(nullptr);
    std::cout<<"{\"events\":[";for(size_t i=0;i<events.size();++i){if(i)std::cout<<',';std::cout<<'[';for(int j=0;j<6;++j){if(j)std::cout<<',';std::cout<<events[i][j];}std::cout<<']';}std::cout<<"],\"metadata\":[";
    for(int i=0;i<4;++i){if(i)std::cout<<',';auto p=objects[i];std::cout<<'['<<p->GetAllFlags()<<','<<mark<Mark1>(p)<<','<<mark<Mark2>(p)<<','<<mark<Mark3>(p)<<",[";SmTArray<SmAttribute*>attrs;p->GetAttributes(attrs);for(ULONG j=0;j<attrs.GetSize();++j){if(j)std::cout<<',';std::cout<<static_cast<SmLongAttribute*>(attrs[j])->GetValue();}std::cout<<"]]";}
    std::cout<<"],\"trees\":";auto cache=static_cast<SmBrepCache*>(SmCacheMgr::GetObjectCache(SM_OC_BREP,b));
    if(!cache)std::cout<<"null";else{std::map<const void*,unsigned>ids;for(unsigned i=0;i<3;++i){ids[v[i]]=i;ids[e[i]->GetCurve()]=i;}ids[face->GetSurface()]=0;std::cout<<'[';std::array<SmTree*,3>trees={cache->GetVertexTree(),cache->GetCurveTree(),cache->GetSurfaceTree()};for(int i=0;i<3;++i){if(i)std::cout<<',';SmTArray<SmObjectList*>lists;require(trees[i]->GetObjectList(lists));std::set<unsigned>found;for(ULONG j=0;j<lists.GetSize();++j)for(auto item=lists[j];item;item=item->m_pNext)if(item->m_pObject)found.insert(ids.at(item->m_pObject));std::cout<<'[';bool first=true;for(auto id:found){if(!first)std::cout<<',';first=false;std::cout<<id;}std::cout<<']';}std::cout<<']';}
    std::cout<<"}\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
