<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Public native construction controls

`cylinder.step` is the unchanged public synthetic
`source/STEP_IMPORT/tests/fixtures/conic_bounds_cylinder.step` from the maintained
usd-brep fork at `d8ed45a`. Its source generator is `cylinder_source()` in
`tools/step_import_native_validation/tests/test_general.py`. No private model
or derived geometry is included.

Independent definition: radius 3 mm, height 7 mm, two vertices, two closed circle
edges and one explicit shared seam, with three directed faces. Its analytic
area is `60*pi` mm², volume `63*pi` mm³ and bounds `[-3,-3,0]..[3,3,7]` mm.
The unique boundary curves have total length `12*pi+7` mm.

The construction regression exercises source interpretation, concrete receiver
retention, native validation, installed-trim measurements and numerical material
observations. These observations do not establish continuous error bounds or
general import acceptance.

`torus-latitude-band.step` is a public synthetic ring-torus cap: major radius
10 mm, minor radius 5 mm, bounded at z=3 mm by coaxial circles of radii 14 mm
and 6 mm and a planar annular cap. The source has two vertices, two closed circle
edges and two faces; it deliberately has no source meridian edge. Both circles,
their vertices and source occurrences survive the derived paired minor-circle cut.
No private input, inventory or derived data was used to construct this fixture.

Let `beta=acos(3/5)`. Independent elementary integrals give material volume
`(500*beta-240)*pi` mm³, total area `(160+200*beta)*pi` mm² and centroid
`z=(2560/3)/(500*beta-240)` mm. Its bounds are `[-14,-14,3]..[14,14,5]` mm.
The cut is a circular arc of length `10*beta` mm through `(10,0,5)`, rather than
the straight chord between its ends. The integration tests also exercise the
complementary long minor-angle band without changing the defining torus/circles.
