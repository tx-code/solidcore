#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Compare actual C++ types/units methods with Rust, using public synthetic text.

Checks full diagnostic provenance, phase visits/counters, typed slots, membership,
exact dimensions and SI metadata. It never treats checkpoint success as part,
geometry or native acceptance. C++ is a compatibility reference, not independent
geometric truth. No customer model paths are scanned and no network is used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

AP203='AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}'
AP214='AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1}'
MBX='AUTOMOTIVE_DESIGN {1 0 10303 214 1 1 1 1}'


def file(data,schema=AP214):
    return ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('public synthetic types/units'),'2;1');\n"
            "FILE_NAME('public.step','',(''),(''),'','','');\nFILE_SCHEMA(('"+schema+"'));\n"
            "ENDSEC;\nDATA;\n"+data+"\nENDSEC;\nEND-ISO-10303-21;\n").encode('ascii')


def norm(value):
    if isinstance(value,list): return [norm(v) for v in value]
    if not isinstance(value,dict): return value
    if set(value)=={'begin','end'}: return [value['begin'],value['end']]
    return {k:norm(v) for k,v in value.items() if k not in {'message'}}


def selection(value):
    if value is None:return None
    value=norm(value)
    for identifier in value['identifiers']: identifier['registry_id']=identifier.get('registry_id') or ''
    profile=value.pop('qualified_profile',value.pop('qualified',None))
    if profile:
        profile={k:profile[k] for k in ['registry_id','profile','express_sha256']}
    return {'identifiers':value['identifiers'],'qualified':profile}


def compare(cpp,rust,path,options=()):
    observations=[]
    for binary in [cpp,rust]:
        p=subprocess.run([str(binary),'--fixture-limits',*options,str(path)],capture_output=True,timeout=30,check=False)
        assert p.returncode in ({0,1,2,3,7} if {'--part-and-closure','--source-semantics'} & set(options) else {0,1,2,3}),f'reader infrastructure/argument failure: {p.returncode}: {p.stderr.decode(errors="replace")}'
        observations.append((p.returncode,json.loads(p.stdout)))
    (c_code,c),(r_code,r)=observations
    assert c_code==r_code,'exit differs'
    for key in ['status','binding','types','unit_dependencies','si_units','dependency_visits','unit_dependency_checks','diagnostics']:
        assert norm(c[key])==norm(r[key]),f'{key} differs: {str(norm(c[key]))[:600]} / {str(norm(r[key]))[:600]}'
    assert selection(c['schema_selection'])==selection(r['schema_selection']),'schema observations differ'
    assert c['part']==r['part']
    ownership=bool({'--part-and-closure','--source-semantics'} & set(options))
    if ownership:
        assert c['property_attachment_checks']==r['property_attachment_checks'],'property visit count differs'
    else:
        assert c['part']=='NotRun'
    assert c['geometry_dependencies']==r['geometry_dependencies']
    if {'--geometry-dependencies','--source-semantics'} & set(options):
        assert c['geometry_dependency_checks']==r['geometry_dependency_checks'],'geometry prerequisite counters differ'
    else:
        assert c['geometry_dependencies']=='NotRun'
    assert r['native_build']=='NotRun' and r['interoperability']=='NotRun' and r['repair']=='None'
    if c['snapshot'] is None: assert r['snapshot'] is None,'partial typed snapshot published'
    else:
        assert r['snapshot'] is not None
        for key in ['registered_types','records']:
            assert norm(c['snapshot'][key])==norm(r['snapshot'][key]),f'typed {key} differs'
        units=dict(r['snapshot']['units']);units.pop('context')
        assert norm(c['snapshot']['units'])==norm(units),'unit metadata differs'
        if ownership:
            assert norm(c['snapshot']['part_definition'])==norm(r['snapshot']['part_definition']),'part definition/roles differ'
    raw=path.read_bytes()
    c_source,r_source=c['source'],r['source']
    assert c_source['status']==r_source['status'],'source inspection differs'
    if r_source['sha256'] is not None:
        assert c_source['input']['sha256']==r_source['sha256']==hashlib.sha256(raw).hexdigest()
    if r_source['raw'] is not None:
        assert c_source['input']['raw'].encode('latin1')==r_source['raw'].encode('latin1')==raw,'source rewritten'
    return r


def unit_cases():
    length="#1=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
    angle="#2=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n"
    mass="#3=(MASS_UNIT() NAMED_UNIT(*) SI_UNIT(.KILO.,.GRAM.));\n"
    for power in ['2.','0.','-1.','0.5','1.','1.E0','1.E-20','9223372036854775808.E-1',
                  '0.E'+'9'*100,'1.E'+'9'*100,'1.E-'+'9'*100]:
        yield length+f'#4=DERIVED_UNIT_ELEMENT(#1,{power});#5=DERIVED_UNIT((#4));'
    for prefix in ['$','.KILO.','.MILLI.']:
        for power in ['0.','-3.']:
            yield mass.replace('.KILO.',prefix)+f'#4=DERIVED_UNIT_ELEMENT(#3,{power});#5=DERIVED_UNIT((#4));'
    yield length+mass+'#4=DERIVED_UNIT_ELEMENT(#1,-3.);#5=DERIVED_UNIT_ELEMENT(#3,1.);#6=DERIVED_UNIT((#4,#5));#7=MEASURE_WITH_UNIT(RATIO_MEASURE(1.),#6);'
    for value in ['0.','-0.','-1.','1.E-999999999999999999999999999999999999']:
        for tag in ['POSITIVE_LENGTH_MEASURE','NON_NEGATIVE_LENGTH_MEASURE','POSITIVE_RATIO_MEASURE','POSITIVE_PLANE_ANGLE_MEASURE']:
            yield length+angle+f'#4=MEASURE_WITH_UNIT({tag}({value}),#1);'
    for tag in ['LENGTH_MEASURE','MASS_MEASURE','RATIO_MEASURE','PLANE_ANGLE_MEASURE','POSITIVE_PLANE_ANGLE_MEASURE','COUNT_MEASURE','NUMERIC_MEASURE','DESCRIPTIVE_MEASURE']:
        value="'text'" if tag=='DESCRIPTIVE_MEASURE' else '-1'
        yield length+f'#4=MEASURE_WITH_UNIT({tag}({value}),#1);'
        yield length+f'#4=VALUE_REPRESENTATION_ITEM(\'unconverted\',{tag}({value}));'
    yield '#1=(CONVERSION_BASED_UNIT(\'cycle\',#2) LENGTH_UNIT() NAMED_UNIT(*));#2=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.),#1);'
    yield length+'#2=DERIVED_UNIT_ELEMENT(#1,2.);#3=DERIVED_UNIT((#2,#2));'
    yield length+'#2=DERIVED_UNIT_ELEMENT(#1,0.5);#3=DERIVED_UNIT_ELEMENT(#1,0.5);#4=DERIVED_UNIT((#2,#3));'
    yield angle+'#3=DERIVED_UNIT_ELEMENT(#2,1.E999999999999999999999999);#4=DERIVED_UNIT((#3));'
    yield length+'#2=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.GRAM.));'
    yield length.replace('.METRE.','.SECOND.')
    yield length+'#4=DERIVED_UNIT_ELEMENT(#1,2.);#5=DERIVED_UNIT((#4));#6=PLANE_ANGLE_MEASURE_WITH_UNIT(RATIO_MEASURE(1.),#5);'
    rational="#1=(BOUNDED_SURFACE() B_SPLINE_SURFACE(1,1,((#2,#2),(#2,#2)),.UNSPECIFIED.,.F.,.F.,.U.) B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.,1.),(1.,1.))) REPRESENTATION_ITEM('') SURFACE());#2=CARTESIAN_POINT('',(0.,0.,0.));"
    yield rational
    yield rational.replace('((#2,#2),(#2,#2))','((#2,#2),(#2,0))')
    yield rational.replace('((#2,#2),(#2,#2))','((#2),(#2,#2))')
    yield "#1=POINT_STYLE('marker',MARKER_TYPE(.RING.),POSITIVE_LENGTH_MEASURE(1.),#2);#2=COLOUR_RGB('colour',1.,0.,0.);"
    trim="#1=LINE('',#2,#4);#2=CARTESIAN_POINT('',(0.,0.,0.));#4=VECTOR('',#5,1.);#5=DIRECTION('',(1.,0.,0.));#3=TRIMMED_CURVE('',#1,(PARAMETER_VALUE(-0.),#2),(PARAMETER_VALUE(1.),#2),.T.,.PARAMETER.);"
    yield trim
    yield trim.replace('PARAMETER_VALUE(-0.),#2','PARAMETER_VALUE(-0.),PARAMETER_VALUE(0.E99)')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cpp',type=Path,required=True)
    parser.add_argument('--rust',type=Path,required=True)
    parser.add_argument('--corpus',type=Path)
    parser.add_argument('--catalog',type=Path,help='Optional actual C++ step_catalog_snapshot export')
    parser.add_argument('--quick',action='store_true',help='Skip exhaustive descriptor/signature mapping cases')
    args=parser.parse_args();cpp=args.cpp.resolve();rust=args.rust.resolve();count=0
    if args.corpus:
        cases=sorted(args.corpus.glob('S203-*/input.step'));assert len(cases)==49
        for case in cases:
            try: compare(cpp,rust,case.resolve(),['--unit-context','12'])
            except AssertionError as error:raise AssertionError(f'public {case.parent.name}: {error}') from error
            count+=1
        print(json.dumps({'frozen_cases':count,'differences':0}),flush=True)
    with tempfile.TemporaryDirectory(prefix='solidcore-types-units-public-') as temp:
        path=Path(temp)/'synthetic.step'
        for i,data in enumerate(unit_cases()):
            path.write_bytes(file(data))
            try:compare(cpp,rust,path)
            except AssertionError as error:raise AssertionError(f'public unit case {i}: {error}') from error
            count+=1
        # Protocol identity recognition is distinct from fixed implementation selection.
        for schema in [AP203,AP214,MBX,'CONFIG_CONTROL_DESIGN','AUTOMOTIVE_DESIGN','AUTOMOTIVE_DESIGN_CC2',
                       'AUTOMOTIVE_DESIGN {1 0 10303 214 3 1 1 1}','AUTOMOTIVE_DESIGN {01 0 10303 214 3 1 1}',
                       'AUTOMOTIVE_DESIGN {1 0 10303 214 2 1 1}','AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {1 0 10303 442 1 1 4}',
                       'UNKNOWN','BAD {1 -1}','BAD {1} trailing','BAD }']:
            path.write_bytes(file('#1=APPLICATION_CONTEXT(\'\');',schema))
            for options in [[],['--general-brep-source']]:compare(cpp,rust,path,options);count+=1
        plane=Path(__file__).resolve().parent.parent/'crates/solidcore-step/tests/fixtures/plane.step'
        original=plane.read_bytes()
        converted=original.replace(b"#10=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));",b"#10=(CONVERSION_BASED_UNIT('inch',#901) LENGTH_UNIT() NAMED_UNIT(#902));\n#900=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.));\n#901=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.0254),#900);\n#902=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);")
        path.write_bytes(converted)
        for options in [['--unit-context','12'],['--unit-context','12','--general-brep-source']]:
            assert compare(cpp,rust,path,options)['status']=='Deferred';count+=1
        if args.catalog and not args.quick:
            catalog=json.loads(args.catalog.read_bytes())
            for profile in catalog['profiles']:
                schema={'Ap203e2Planar':AP203,'Ap203e2GeneralSource':AP203,'Ap214e3Source':AP214,'Ap214MbxSource':MBX,'Ap203N916Source':'CONFIG_CONTROL_DESIGN'}[profile['kind']]
                options=[] if profile['kind']=='Ap203e2Planar' else ['--general-brep-source']
                for declaration in profile['declarations']:
                    path.write_bytes(file('#1='+declaration['name']+'();',schema))
                    try:compare(cpp,rust,path,options)
                    except AssertionError as error:raise AssertionError(f'{profile["kind"]} {declaration["name"]}: {error}') from error
                    count+=1
                for signature in profile['complex_signatures']:
                    path.write_bytes(file('#1=('+' '.join(name+'()' for name in signature)+');',schema));compare(cpp,rust,path,options);count+=1
                print(json.dumps({'catalog':profile['kind'],'compared_so_far':count,'differences':0}),flush=True)
        # Exact visit boundaries include the attempted over-budget visit.
        for options,content in [([],file('#1=APPLICATION_CONTEXT(\'\');',AP203)),([],file(next(iter(unit_cases()))))]:
            path.write_bytes(content);good=compare(cpp,rust,path,options);visits=good['dependency_visits']
            for n in [0,visits-1,visits,10_000_001]:compare(cpp,rust,path,[*options,'--dependency-visits',str(n)]);count+=1
    print(json.dumps({'compared':count,'differences':0,'scope':'strict types/unit prerequisites; ownership/geometry/native NotRun'}))


if __name__=='__main__':main()
