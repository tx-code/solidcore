// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Test-only original API observer: custom qualities and complete diagnostics.
// No predicate, numeric operation, phase, budget or original assertion is changed.
#include "StepPlanar.h"
#include "StepSourceReport.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#endif

namespace step_semantics {
Outcome CaptureCheck(const step_source::InspectOutcome& source, Limits limits,
    SourcePolicy policy=SourcePolicy::Strict, GeometryScope scope=GeometryScope::ExplicitOuterBounds,
    SchemaInterpretation interpretation=SchemaInterpretation::DeclaredOnly) {
    return Check(source,limits,policy,scope,interpretation);
}
}
namespace step_planar {
Outcome CaptureCheck(const step_semantics::Outcome& semantic, Quality quality, Limits limits) {
    if (semantic.snapshot) {
        const char* directory=std::getenv("SOLIDCORE_PLANAR_PUBLIC_CAPTURE");
        if(!directory)throw std::runtime_error("explicit public capture directory required");
        static std::size_t serial=0; // Test process only; the original suite is single-threaded.
        const auto name=std::to_string(serial++);
        const auto root=std::filesystem::path(directory);
        std::ofstream input(root/(name+".step"),std::ios::binary); input<<semantic.snapshot->Source().Bytes();
        std::string request;
        step_source::report::Writer::Publish(16384,request,[&](auto& w){
            w.Add("{\"cpp_flags\":[\"--fixture-limits\",\"--quality\",");w.String(quality.id);
            w.Add(",\"--consistency-mm\",");w.String(std::to_string(quality.consistencyMm.Numerator())+"/"+std::to_string(quality.consistencyMm.Denominator()));
            w.Add(",\"--max-planar-work\",");w.String(std::to_string(limits.work));w.Add("]}\n");
        });
        std::ofstream options(root/(name+".request.json"),std::ios::binary);options<<request;
        if(!input||!options)throw std::runtime_error("public capture write failed");
    }
    return Check(semantic,std::move(quality),limits);
}
}
#define Check CaptureCheck
#define main OriginalPlanarContractMain
#include "StepPlanarTests.cpp"
#undef main
#undef Check

void FullDiagnostic(step_source::report::Writer& w,const step_semantics::Diagnostic& d) {
    w.Add("{\"source\":{\"key\":");w.String(d.source.key);w.Add(",\"stage\":");w.String(d.source.stage);
    w.Add(",\"record_id\":");w.String(d.source.recordId);w.Add(",\"component\":");w.String(d.source.component);
    w.Add(",\"parameter_path\":");w.Path(d.source.parameterPath);w.Add(",\"target_id\":");w.String(d.source.targetId);
    w.Add(",\"primary\":");w.Range(d.source.primary);w.Add(",\"related\":[");
    bool comma=false;for(auto span:d.source.related){if(comma)w.Add(",");comma=true;w.Range(span);}w.Add("]}");
    for(auto pair:std::initializer_list<std::pair<const char*,std::string>>{{"classification",d.classification},{"field",d.field},{"declared_in",d.declaredIn},{"declared_type",d.declaredType},{"expected",d.expected},{"actual",d.actual},{"responsibility",d.responsibility}}){w.Add(",");w.String(pair.first);w.Add(":");w.Utf8String(pair.second);}
    w.Add("}");
}
int main(int argc,char**argv){
    try {
#if defined(_WIN32)
        if(_setmode(_fileno(stdout),_O_BINARY)==-1)return 6;
#endif
        if(argc==3&&std::string(argv[1])=="--capture")return OriginalPlanarContractMain(argc-1,argv+1);
        using namespace step_planar;
        auto sourceLimits=step_source::Limits::PlanarFixtures();step_semantics::Limits semanticLimits;Limits limits;
        Quality quality=Quality::PlanarFixture();bool selected=false,qualitySelected=false;std::string path;
        for(int i=1;i<argc;++i){const std::string arg(argv[i]);
            if(arg=="--fixture-limits")selected=true;
            else if(arg=="--quality"&&i+1<argc){quality.id=argv[++i];qualitySelected=true;}
            else if(arg=="--consistency-mm"&&i+1<argc){std::string value(argv[++i]);auto slash=value.find('/');quality.consistencyMm=slash==std::string::npos?Rational::Decimal(value):Rational(std::stoll(value.substr(0,slash)),std::stoll(value.substr(slash+1)));}
            else if(arg=="--max-planar-work"&&i+1<argc)limits.work=std::stoull(argv[++i]);
            else if(arg=="--max-semantic-visits"&&i+1<argc)semanticLimits.dependencyVisits=std::stoull(argv[++i]);
            else if(arg=="--max-report-bytes"&&i+1<argc)sourceLimits.reportBytes=std::stoull(argv[++i]);
            else if(path.empty()&&!arg.starts_with("--"))path=arg;
            else throw std::invalid_argument("test reference arguments");
        }
        if(!selected||!qualitySelected||path.empty())return 64;
        const auto source=step_source::InspectFile(path,sourceLimits);const auto semantic=step_semantics::Check(source,semanticLimits);
        const auto result=Check(semantic,quality,limits);std::string original,output;
        if(!InspectJson(source,semantic,result,original))return 5;
        if(!step_source::report::Writer::Publish(sourceLimits.reportBytes,output,[&](auto&w){
            w.Add("{\"original_wire\":");w.Add(original);w.Add(",\"full_api_diagnostics\":[");
            bool comma=false;for(auto&d:result.diagnostics){if(comma)w.Add(",");comma=true;FullDiagnostic(w,d);}w.Add("]}\n");
        }))return 5;
        std::cout<<output;if(!std::cout)return 6;
        switch(result.status){case Status::Checked:return 0;case Status::Rejected:return 1;case Status::Unsupported:return 2;case Status::ResourceLimit:return 3;case Status::Deferred:return 7;case Status::Indeterminate:return 8;
            case Status::UpstreamFailure: if(semantic.status==step_semantics::Status::Unsupported||source.status==step_source::Status::Unsupported)return 2;
                if(semantic.status==step_semantics::Status::ResourceLimit||source.status==step_source::Status::ResourceLimit)return 3;return 1;}
        return 6;
    }catch(const std::invalid_argument&){return 64;}catch(const std::exception&error){std::cerr<<error.what()<<'\n';return 6;}
}
