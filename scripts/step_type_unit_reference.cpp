// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Test-only entrypoint appended to an access-enabled copy of StepSemantics.cpp.
// Existing checker bodies, failure paths, visit counters and catalogs are unchanged.
// This deliberately stops at types/units: part ownership and geometry are NotRun.
#include "StepSourceReport.h"
#include "StepSemanticReport.h"
#include <iostream>
#include <fstream>

void CheckpointJson(step_source::report::Writer& w, const step_source::InspectOutcome& source, const step_semantics::Outcome& result)
{
    using namespace step_semantics;
    w.Add("{\"schema\":\"cpp.step.types-units.checkpoint.v1\",\"status\":"); w.String(StatusName(result.status));
    w.Add(",\"schema_selection\":"); report::WriteSchemaSelection(w,result.schemaSelection);
    w.Add(",\"binding\":");w.String(result.binding);w.Add(",\"types\":");w.String(result.types);
    w.Add(",\"unit_dependencies\":");w.String(result.unitDependencies);w.Add(",\"si_units\":");w.String(result.siUnits);
    w.Add(",\"part\":\"NotRun\",\"geometry_dependencies\":\"NotRun\",\"dependency_visits\":");w.Number(result.dependencyVisits);
    w.Add(",\"unit_dependency_checks\":{\"total\":");w.Number(result.unitDependencyChecks.total);
    w.Add(",\"unit_definitions\":");w.Number(result.unitDependencyChecks.unitDefinitions);
    w.Add(",\"type_predicates\":");w.Number(result.unitDependencyChecks.typePredicates);w.Add("},\"diagnostics\":[");
    bool comma=false;
    for (const auto& d:result.diagnostics)
    {
        if(comma)w.Add(",");comma=true;
        w.Add("{\"source\":{\"key\":");w.String(d.source.key);w.Add(",\"stage\":");w.String(d.source.stage);
        w.Add(",\"primary\":");w.Range(d.source.primary);w.Add(",\"record_id\":");w.String(d.source.recordId);
        w.Add(",\"component\":");w.String(d.source.component);w.Add(",\"parameter_path\":");w.Path(d.source.parameterPath);
        w.Add(",\"target_id\":");w.String(d.source.targetId);w.Add(",\"related\":[");
        bool related=false;for(auto s:d.source.related){if(related)w.Add(",");related=true;w.Range(s);}w.Add("]}");
        for (const auto& pair:std::initializer_list<std::pair<const char*,std::string>>{{"classification",d.classification},{"field",d.field},{"declared_in",d.declaredIn},{"declared_type",d.declaredType},{"expected",d.expected},{"actual",d.actual},{"responsibility",d.responsibility}})
        {w.Add(",");w.String(pair.first);w.Add(":");w.Utf8String(pair.second);}w.Add("}");
    }
    w.Add("],\"snapshot\":");
    if(!result.snapshot){w.Add("null");}
    else
    {
        const auto& snapshot=*result.snapshot;const auto& raw=snapshot.Source();
        w.Add("{\"registered_types\":[");comma=false;for(auto name:snapshot.RegisteredTypes()){if(comma)w.Add(",");comma=true;w.String(name);}w.Add("],\"records\":[");comma=false;
        for(std::size_t r=0;r<raw.Records().size();++r)
        {
            if(comma)w.Add(",");comma=true;w.Add("{\"id\":");w.String(raw.Records()[r].id);w.Add(",\"types\":[");
            bool types=false;for(const auto& type:snapshot.TypesOf(r)){if(types)w.Add(",");types=true;w.String(type);}w.Add("],\"attributes\":[");
            bool fields=false;for(std::size_t c=0;c<raw.Records()[r].components.size();++c)for(std::size_t p=0;p<raw.Records()[r].components[c].parameters.size();++p)
            {
                if(fields)w.Add(",");fields=true;const auto a=snapshot.Attribute(r,c,p);
                w.Add("{\"component\":");w.Number(c);w.Add(",\"parameter\":");w.Number(p);
                for(const auto& pair:std::initializer_list<std::pair<const char*,std::string_view>>{{"name",a.name},{"declared_in",a.declaredIn},{"declared_type",a.declaredType},{"redeclared_by",a.redeclaredBy},{"aggregate",a.aggregate}}){w.Add(",");w.String(pair.first);w.Add(":");w.String(pair.second);}
                w.Add(",\"optional\":");w.Bool(a.optional);w.Add(",\"derived_slot\":");w.Bool(a.derivedSlot);
                w.Add(",\"minimum\":");w.Number(a.minimum);w.Add(",\"maximum\":");if(a.maximum)w.Number(*a.maximum);else w.Add("null");
                w.Add(",\"unique_elements\":");w.Bool(a.uniqueElements);w.Add(",\"nested_aggregates\":[");bool nested=false;
                for(const auto& d:a.nestedAggregates){if(nested)w.Add(",");nested=true;w.Add("{\"aggregate\":");w.String(d.aggregate);w.Add(",\"minimum\":");w.Number(d.minimum);w.Add(",\"maximum\":");if(d.maximum)w.Number(*d.maximum);else w.Add("null");w.Add(",\"unique_elements\":");w.Bool(d.uniqueElements);w.Add("}");}w.Add("]}");
            }w.Add("]}");
        }
        w.Add("],\"units\":{\"length_unit\":");const auto& units=snapshot.UnitContext();
        const bool context=result.siUnits=="Passed"||result.siUnits=="Deferred";
        if(context)w.Number(units.lengthUnit);else w.Add("null");w.Add(",\"plane_angle_unit\":");if(context)w.Number(units.planeAngleUnit);else w.Add("null");
        w.Add(",\"solid_angle_unit\":");if(units.solidAngleUnit)w.Number(*units.solidAngleUnit);else w.Add("null");
        w.Add(",\"length_power10_mm\":");if(units.lengthPower10Mm)w.Add(std::to_string(*units.lengthPower10Mm));else w.Add("null");
        w.Add(",\"si_definitions\":[");comma=false;
        for(const auto& si:units.siDefinitions)
        {
            if(comma)w.Add(",");comma=true;w.Add("{\"record_index\":");w.Number(si.recordIndex);
            w.Add(",\"quantity\":");w.String(si.quantity);w.Add(",\"si_name\":");w.String(si.siName);w.Add(",\"prefix\":");w.String(si.prefix);
            w.Add(",\"canonical_power10\":");w.Add(std::to_string(si.canonicalPower10));w.Add(",\"dimensions\":[");bool dim=false;
            for(auto n:si.dimensions){if(dim)w.Add(",");dim=true;w.Add(std::to_string(n));}w.Add("]}");
        }
        w.Add("],\"dimension_definitions\":[");comma=false;
        for(const auto& definition:units.dimensionDefinitions)
        {
            if(comma)w.Add(",");comma=true;w.Add("{\"record_index\":");w.Number(definition.recordIndex);w.Add(",\"nominal\":");w.String(definition.nominal);w.Add(",\"dimensions\":[");bool dim=false;
            for(const auto& n:definition.dimensions){if(dim)w.Add(",");dim=true;w.Add("{\"numerator\":");w.String(n.numerator);w.Add(",\"denominator\":");w.String(n.denominator);w.Add("}");}w.Add("]}");
        }w.Add("]}}");
    }
    w.Add(",\"source\":");step_source::report::WriteJson(w,source);w.Add("}\n");
}

int main(int argc, char** argv)
{
    using namespace step_semantics;
    try
    {
        std::string input, context;
        auto sourceLimits = step_source::Limits::PlanarFixtures();
        Limits limits;
        auto scope = GeometryScope::ExplicitOuterBounds;
        bool explicitLimits = false;
        for (int i = 1; i < argc; ++i)
        {
            std::string arg(argv[i]);
            if (arg == "--fixture-limits") explicitLimits = true;
            else if (arg == "--general-brep-source") scope = GeometryScope::GeneralBrep;
            else if (arg == "--unit-context" && i + 1 < argc) context = argv[++i];
            else if (arg == "--dependency-visits" && i + 1 < argc) limits.dependencyVisits = std::stoull(argv[++i]);
            else if (arg == "--input-bytes" && i + 1 < argc) sourceLimits.inputBytes = std::stoull(argv[++i]);
            else if (arg == "--report-bytes" && i + 1 < argc) sourceLimits.reportBytes = std::stoull(argv[++i]);
            else if (input.empty() && !arg.starts_with("--")) input = arg;
            else throw std::invalid_argument("test checkpoint arguments");
        }
        if (!explicitLimits || input.empty()) return 64;
        auto source = step_source::InspectFile(input, sourceLimits);
        Outcome result;
        result.limits = limits;
        result.geometryScope = scope;
        if (!source.source || limits.dependencyVisits > 10000000)
            result = Check(source, limits, SourcePolicy::Strict, scope);
        else
        {
            auto data = std::make_shared<SemanticData>();
            data->source = source.source;
            data->geometryScope = scope;
            Checker checker(data, result);
            try
            {
                checker.RunPhase(Checker::Phase::Binding, [&] { checker.Binding(); });
                checker.RunPhase(Checker::Phase::Types, [&] {
                    checker.Mapping(); checker.CheckForeignDeclarations(); checker.Fields();
                });
                if (data->profile.kind != ProfileKind::Ap203e2Planar)
                {
                    checker.RunPhase(Checker::Phase::UnitDependencies, [&] {
                        if (auto issue = CheckAp214UnitDependencies(*data, [&](step_source::Span span) { checker.Visit(span); }, result.unitDependencyChecks))
                        {
                            checker.State(Checker::Phase::UnitDependencies) = Checker::FailureProgress(issue->status);
                            throw Failure { issue->status, std::move(issue->diagnostic) };
                        }
                    });
                }
                if (!context.empty())
                {
                    checker.m_context = IndexOf(*data, context);
                    if (!IsA(*data, checker.m_context, "GEOMETRIC_REPRESENTATION_CONTEXT"))
                        throw std::invalid_argument("context must be a typed geometric representation context");
                    checker.RunPhase(Checker::Phase::Units, [&] { checker.UnitContext(); });
                    if (!data->units.lengthPower10Mm)
                        checker.State(Checker::Phase::Units) = Checker::Progress::Deferred;
                }
                result.status = context.empty() || data->units.lengthPower10Mm ? Status::Checked : Status::Deferred;
                result.snapshot = std::make_shared<const Snapshot>(data);
            }
            catch (const Failure& failure)
            {
                result.status = failure.status;
                result.diagnostics.push_back(failure.diagnostic);
            }
            checker.PublishProgress();
        }
        std::string json;
        if (!step_source::report::Writer::Publish(sourceLimits.reportBytes,json,[&](auto& w){CheckpointJson(w,source,result);})) return 5;
        std::cout << json;
        if (!std::cout) return 6;
        if (result.status == Status::SourceFailure) return 1;
        if (result.status == Status::Rejected) return 1;
        if (result.status == Status::Unsupported) return 2;
        if (result.status == Status::ResourceLimit) return 3;
        return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 64; }
}
