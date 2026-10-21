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
retention, native validation and installed-trim measurements. It does not claim
that the still-unimplemented material measurements or general import acceptance
have been qualified by these analytic values.
