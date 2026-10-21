// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Public synthetic CrvOnSurf virtual dispatch against the maintained SDK.
#include "SmAxis2Placement.h"
#include "SmBSplineSurface.h"
#include "SmContext.h"
#include "SmCrvOnSurf.h"
#include "SmDatabaseIO.h"
#include "SmLine.h"
#include "SmPlane.h"
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

static void require(SmStatus s) { if(s!=SM_SUCCESS) throw std::runtime_error("native status "+std::to_string(s)); }
static void vec(const SmVector3d& v) { std::cout<<'['<<v.x<<','<<v.y<<','<<v.z<<']'; }
static void observe(SmCrvOnSurf& c,double t) {
    std::array<SmVector3d,4> d; require(c.Evaluate(t,3,TRUE,d.data(),FALSE));
    std::array<SmVector3d,2> u,v; require(c.EvaluateSurface1stDerivDerivs(t,1,TRUE,u.data(),v.data()));
    SmVector2d uv[2]; SmVector3d n; require(c.EvaluateSurfaceNormal(t,TRUE,uv,n));
    auto domain=c.GetNaturalInterval();
    std::cout<<"{\"surface_type\":"<<c.GetBaseSurface()->GetType()<<",\"interval\":["<<domain.GetMin()<<','<<domain.GetMax()<<"],\"derivatives\":[";
    for(size_t i=0;i<d.size();++i) { if(i)std::cout<<','; vec(d[i]); }
    std::cout<<"],\"normal\":";vec(n);std::cout<<",\"uv\":[["<<uv[0].x<<','<<uv[0].y<<"],["<<uv[1].x<<','<<uv[1].y<<"]],\"surface_derivatives\":[";
    vec(u[0]);std::cout<<',';vec(u[1]);std::cout<<',';vec(v[0]);std::cout<<',';vec(v[1]);std::cout<<"]}";
}
int main(int argc,char** argv) { try {
    if(argc<7)throw std::runtime_error("shape owner action parameter scale offset");
    std::cout<<std::setprecision(17);
    const std::string shape=argv[1],action=argv[3]; const ULONG flag=std::stoul(argv[2]);
    const double t=std::stod(argv[4]),s=std::stod(argv[5]),o=std::stod(argv[6]);
    SmContext k; SmExtent2d domain(0,0,1,1);SmSurface* surface=nullptr;
    if(shape=="bilinear") {
        SmBSplineSurface* b=nullptr;
        require(SmBSplineSurface::CreateBilinearSurface(k,SmPoint3d(o,o,o),SmPoint3d(o+s,o,o),SmPoint3d(o,o+s,o),SmPoint3d(o+s,o+s,o+s),b));surface=b;
    } else {
        auto x=shape=="rotated_plane"?SmVector3d(0,1,0):SmVector3d(1,0,0);
        auto y=shape=="rotated_plane"?SmVector3d(0,0,1):SmVector3d(0,1,0);
        surface=new(&k) SmPlane(SmPoint3d(o,o,o),x,y,SmVector2d(s,2*s),domain,&k);
    }
    SmLine* uv=nullptr;require(SmLine::CreateLineSegment(k,2,SmPoint2d(.125,.25),SmPoint2d(.875,.75),uv));
    {
        // Flag3 transfers both heap allocations to c; other flags leave the
        // originals alive through the observation and destroy only their copies.
        SmCrvOnSurf c(*uv,*surface,&domain,flag,&k);
        SmAxis2Placement transform(SmPoint3d(3,-2,1),SmVector3d(0,1,0),SmVector3d(-1,0,0));
        SmVector3d scale=action=="nonuniform"?SmVector3d(2,3,4):SmVector3d(2,2,2);
        SmStatus status=SM_SUCCESS;
        if(action=="transform"||action=="nonuniform")status=c.Transform(transform,&scale);
        if(action=="identity") { SmAxis2Placement identity;require(c.Transform(identity,nullptr)); }
        std::unique_ptr<SmCurve> copied;
        SmCrvOnSurf* target=&c;
        if(action=="copy"||action=="copy_transform") {
            SmCurve* p=nullptr;require(c.Copy(k,p));copied.reset(p);target=static_cast<SmCrvOnSurf*>(p);
            if(action=="copy_transform")status=target->Transform(transform,&scale);
        }
        std::unique_ptr<SmCurve> restored;std::string bytes;
        if(action=="binary"||action=="ascii") {
            const auto type=action=="binary"?SM_BINARY:SM_ASCII;
            // Primitive widths and nested record version are BOTH explicit.
            // Native default version30 reads Windows long as 32bit, whereas the
            // current writer always emits 64bit longs (database version40).
            std::ostringstream out;out<<std::setprecision(17);SmDatabaseIO writer(type,static_cast<std::ostream*>(&out));writer.SetVersionNum(40);require(c.WriteToDB(writer,40));bytes=out.str();
            std::istringstream in(bytes);SmDatabaseIO reader(type,static_cast<std::istream*>(&in));reader.SetVersionNum(40);
            SmCurve* p=nullptr;require(SmCrvOnSurf::ReadFromDB(SmCrvOnSurf_TYPE,reader,3,k,p,40));restored.reset(p);target=static_cast<SmCrvOnSurf*>(p);
        }
        std::cout<<"{\"status\":";if(status==SM_SUCCESS)std::cout<<"null";else std::cout<<status;
        std::cout<<",\"result\":";observe(*target,t);std::cout<<",\"original\":";observe(c,t);
        if(!bytes.empty()) {std::cout<<",\"archive_hex\":\"";const char* h="0123456789abcdef";for(unsigned char ch:bytes)std::cout<<h[ch>>4]<<h[ch&15];std::cout<<'"';}
        std::cout<<"}\n";
    }
    if(flag!=3) {delete uv;delete surface;}
    return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
