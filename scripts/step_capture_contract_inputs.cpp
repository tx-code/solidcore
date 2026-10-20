// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Test application only: retain PUBLIC SYNTHETIC inputs constructed by the
// existing C++ semantic contract tests. No expected answer is exported as an
// observation, and the original Check implementation still runs unchanged.
#include "StepSemantics.h"
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <stdexcept>

namespace step_semantics
{
Outcome CaptureCheck(const step_source::InspectOutcome& source, Limits limits,
    SourcePolicy policy=SourcePolicy::Strict,
    GeometryScope scope=GeometryScope::ExplicitOuterBounds,
    SchemaInterpretation interpretation=SchemaInterpretation::DeclaredOnly)
{
    if(source.captured && policy==SourcePolicy::Strict &&
       source.headerNamePolicy==step_source::HeaderNamePolicy::StrictAscii &&
       source.dataStringPolicy==step_source::DataStringPolicy::StrictAscii &&
       source.requestedCarrierInterpretation==step_source::CarrierInterpretation::DeclaredOnly &&
       (!source.source || !source.source->AppliedCarrierAssumption()) &&
       interpretation==SchemaInterpretation::DeclaredOnly)
    {
        const char* directory=std::getenv("SOLIDCORE_PUBLIC_CONTRACT_CAPTURE");
        if(!directory)throw std::runtime_error("explicit public capture directory required");
        const std::filesystem::path root(directory);
        const auto name=source.sha256+"-"+std::to_string(static_cast<int>(scope))+"-"+std::to_string(limits.dependencyVisits);
        std::ofstream input(root/(name+".step"),std::ios::binary);input<<*source.captured;
        std::ofstream options(root/(name+".options"),std::ios::binary);
        options<<(scope==GeometryScope::GeneralBrep?"general":"planar")<<'\n'<<limits.dependencyVisits<<'\n';
        if(!input||!options)throw std::runtime_error("public fixture capture write failed");
    }
    return Check(source,limits,policy,scope,interpretation);
}
}

#define Check CaptureCheck
#include "StepSemanticsTests.cpp"
