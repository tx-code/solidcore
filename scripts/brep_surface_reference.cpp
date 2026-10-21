// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Public synthetic point grids/control nets; actual SDK queries/AssertValid.
#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmBrepData.h"
#include "SmBSplineSurface.h"
#include "SmContext.h"
#include "SmFace.h"
#include "SmPlane.h"
#include "SmPointSet.h"
#include <nurbs.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

static void require(SmStatus s){if(s!=SM_SUCCESS)throw std::runtime_error("construction status");}
static const char* boolean(SmBoolean b){return b?"true":"false";}
static void quote(const wchar_t* s){if(!s){std::cout<<"null";return;}std::cout<<'"';
    for(;*s;++s){if(*s==L'"'||*s==L'\\')std::cout<<'\\'<<char(*s);else if(*s==L'\n')std::cout<<"\\n";else if(*s==L'\r')std::cout<<"\\r";else std::cout<<char(*s);}std::cout<<'"';}
static std::vector<double> knots(size_t count,size_t degree){std::vector<double> u(degree+1,0.);
    for(size_t i=1;i<count-degree;++i)u.push_back(double(i)/double(count-degree));
    u.insert(u.end(),degree+1,1.);return u;}
static SmSurface* fixture(SmContext& c,const std::string& name,double scale,double offset){
    if(name=="plane")return new(&c) SmPlane(SmPoint3d(offset,offset,offset),SmVector3d(1,0,0),SmVector3d(0,1,0),SmVector2d(scale,scale),SmExtent2d(0,0,1,1),&c);
    size_t nu=3,nv=3,p=2,q=2;
    if(name=="bilinear"){nu=nv=2;p=q=1;}
    else if(name=="pole_u"){nu=3;nv=2;p=q=1;}
    else if(name=="pole_v"){nu=nv=3;p=2;q=1;}
    else if(name=="row_zero"||name=="late_row_zero"){nu=5;nv=3;}
    else if(name=="false_pole_u"||name=="false_pole_v"){nu=nv=4;}
    else if(name=="mult_u"){nu=6;nv=3;}
    else if(name=="mult_v"){nu=nv=6;}
    std::vector<std::vector<gw_CPOINT>> rows(nu,std::vector<gw_CPOINT>(nv));std::vector<gw_CPOINT*> pointers;
    for(size_t i=0;i<nu;++i){for(size_t j=0;j<nv;++j){double x=double(i),y=double(j),z=0.;
        if(name=="point")x=y=z=0.;else if(name=="curve"){x=double(i+j);y=0.;}
        else if(name=="fold_u")x=i==1?1.:0.;else if(name=="fold_v")y=j==1?1.:0.;
        else if((name=="row_zero"&&i==1)||(name=="late_row_zero"&&i==2)||(name=="pole_u"&&i==1)||(name=="pole_v"&&j==1))x=y=z=0.;
        else if(name=="repeat"&&j==1)y=0.;
        if(name=="false_pole_u"||name=="false_pole_v"){
            size_t a=name=="false_pole_v"?j:i,b=name=="false_pole_v"?i:j;
            if((a==1&&b<=2)||(a==2&&(b==1||b==2)))x=y=z=0.;
            if(a==2&&b==0){x=name=="false_pole_u"?10.:0.;y=name=="false_pole_v"?10.:0.;z=0.;}}
        x=x*scale+offset;y=y*scale+offset;z=z*scale+offset;
        double w=name.find("rational")!=std::string::npos?2.+double(i+j):-std::numeric_limits<double>::max();
        auto& cp=rows[i][j];cp.x=w>0?x*w:x;cp.y=w>0?y*w:y;cp.z=w>0?z*w:z;cp.w=w;
    }pointers.push_back(rows[i].data());}
    auto u=name=="mult_u"?std::vector<double>{0,0,0,.5,.5,.5,1,1,1}:knots(nu,p);
    auto v=name=="mult_v"?std::vector<double>{0,0,0,.5,.5,.5,1,1,1}:knots(nv,q);
    return new(&c) SmBSplineSurface(ULONG(nu-1),ULONG(nv-1),short(p),short(q),ULONG(u.size()-1),ULONG(v.size()-1),pointers.data(),u.data(),v.data());
}
int main(int argc,char**argv){try{
    if(argc<4)throw std::runtime_error("family shape selection [action] [scale] [offset]");
    std::cout<<std::setprecision(17);
    const std::string family=argv[1],name=argv[2],selection=argv[3],action=argc>4?argv[4]:"none";
    const double scale=argc>5?std::stod(argv[5]):1.,offset=argc>6?std::stod(argv[6]):0.;
    SmContext context;
    if(family=="grid"){
        SmTArray<SmPoint3d> points;
        for(int i=0;i<3;++i)for(int j=0;j<3;++j){double x=name=="row_fold"&&j==2?0.:double(j),y=name=="col_fold"&&i==2?0.:double(i),z=0.;
            if(name=="point")x=y=0.;if(name=="repeat"&&j==2)x=1.;
            if(name!="short_row"||i!=2||j!=2)points.Add(SmPoint3d(x*scale+offset,y*scale+offset,z*scale+offset));}
        int flags=std::stoi(selection);
        SmPointGrid grid(points,3,1e-6,(flags&1)!=0,(flags&2)!=0,UNSURE,nullptr,FALSE);
        std::array<SmBoolean,8> values{grid.GetCoincidentCol(),grid.GetCoincidentRow(),grid.GetMonotonicCol(),grid.GetMonotonicRow()};
        grid.SetClosedCol((flags&1)==0);grid.SetClosedRow((flags&2)==0);if(action=="clear")grid.ClearCache();
        values[4]=grid.GetCoincidentCol();values[5]=grid.GetCoincidentRow();values[6]=grid.GetMonotonicCol();values[7]=grid.GetMonotonicRow();
        std::cout<<"{\"grid\":[";for(size_t i=0;i<values.size();++i){if(i)std::cout<<',';std::cout<<boolean(values[i]);}
        std::cout<<"],\"good_row_size\":"<<boolean(grid.IsGoodRowSize())<<"}\n";return 0;
    }
    SmBrepConstructor builder;auto b=builder.StartBrep(context);SmShell *outer=nullptr,*inner=nullptr;
    require(builder.StartShell(outer,inner));auto face=builder.StartFace(SM_OT_SAME);
    auto surface=fixture(context,name,scale,offset);require(builder.SetFaceSurface(surface,SmExtent2d(0,0,1,1)));
    require(builder.EndFace());require(builder.EndShell(FALSE));require(builder.EndBrep());
    auto bsp=static_cast<SmBSplineSurface*>(surface);const bool before=bsp->GetGwNurbPointer()!=nullptr;
    if(family=="query"){
        std::array<SmBoolean,4> values{bsp->HasKnotMultiplicityGreaterThanDegree(),bsp->HasRepeatedEndControlPoints(1e-6),bsp->HasInternalPole(1e-6),bsp->HasDegenerateControlPointRow(1e-6)};
        std::cout<<"{\"queries\":[";for(size_t i=0;i<4;++i){if(i)std::cout<<',';std::cout<<boolean(values[i]);}
        std::cout<<"],\"before_nurb\":"<<boolean(before)<<",\"after_nurb\":"<<boolean(bsp->GetGwNurbPointer()!=nullptr)<<"}\n";return 0;
    }
    SmAssertArray reports;SmTArray<ULONG> requests;requests.Add(0);
    auto level=selection=="given"?SM_LEVEL_GIVEN:selection=="two"?SM_LEVEL_2:selection=="one"?SM_LEVEL_1:selection=="heal"?SM_LEVEL_HEAL:SM_LEVEL_0;
    auto target=selection=="no_reports"?nullptr:&reports;
    auto call=[&]{return surface->AssertValid(target,level,SM_NO_WALK,selection=="given"?&requests:nullptr);};
    bool valid=call();if(selection=="repeat")valid=call();
    std::cout<<"{\"valid\":"<<boolean(valid)<<",\"reports\":[";
    for(ULONG i=0;i<reports.GetSize();++i){if(i)std::cout<<',';auto r=reports[i];
        if(r->m_pOwner!=surface)throw std::runtime_error("unexpected report owner");
        std::cout<<'['<<r->m_lReportingType<<','<<r->m_lListIndex<<','<<r->m_lTestIndex<<','<<r->m_eAssertType<<','<<(1ULL<<63)<<','<<r->m_lOwnerType<<',';
        quote(r->m_pName);std::cout<<',';quote(r->m_pMessage);std::cout<<','<<r->m_sTol3d<<','<<r->m_dValue<<',';quote(r->m_sOwnerTypeString);std::cout<<']';}
    std::cout<<"],\"last_add\":"<<boolean(reports.GetLastAdd())<<",\"before_nurb\":"<<boolean(before)<<",\"after_nurb\":"<<boolean(bsp->GetGwNurbPointer()!=nullptr)<<"}\n";
    return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
