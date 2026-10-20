# Explicit source carrier policies

This ports the opaque/legacy policies in `STEP_SOURCE/inc/StepSource.h`,
`src/StepSource.cpp` and the report/CLI contracts at source commit
`d8ed45ab85d52dba07660cafdceacafdca7217ed`. Policies capture evidence; they do not
authenticate declarations, decode opaque text, approve interoperability or build
BRep. `inspect(bytes, limits)` keeps strict defaults; `inspect_with_options`
accepts three independent explicit choices:

| Option | Alternative and boundary |
|---|---|
| `HeaderNamePolicy` | `PreserveOpaqueBytes` applies only to `HEADER.FILE_NAME.name` |
| `DataStringPolicy` | `PreserveOpaqueBytes` retains non-ASCII DATA STRING tokens |
| `CarrierInterpretation` | `Level1AsCorrectedEncoding` interprets declaration `1` with the corrected grammar; `Level1AsCorrectedEncodingWithLowercaseHex` additionally admits DATA X/X2/X4 numeric `a`–`f` after that assumption is actually applied |

The source CLI accepts `--preserve-opaque-header-name`,
`--preserve-opaque-data-strings` and one of `--interpret-level1-as-corrected` /
`--interpret-level1-with-lowercase-hex`. Conflicting choices are invalid arguments.
The semantic CLI enables legacy interpretation only with explicit general
[source interoperability](INTEROPERABILITY.md), including its final approval.

## Source and evidence ownership

Opaque values have `ValueKind::OpaqueString`, an original token/span and no
interpreted text: `Value::interpreted_text()` returns `None`, JSON `text` is null.
Empty internal storage must not become an effective empty name. No locale, UTF-8,
code page or normalization is inferred. An opaque token's backslash may be a
multibyte trail and is not decoded as a directive. ASCII-only tokens retain the
original decoder and diagnostic/budget precedence. String budgets count original
token bytes, including delimiters/layout, rather than decoded characters.

DEL/unsupported controls and high bytes outside permitted tokens remain refused.
Whole-carrier scans include comments/identifiers/other HEADER fields, run before
publication and remain linear in bytes plus spans. The HEADER-only exception is
checked before DATA; DATA preservation checks completed spans before reference
resolution. Every syntax/reference/resource failure withholds the partial graph.

JSON `raw` projects each byte to the same-numbered U+0000–U+00FF value, matching
C++ raw fields after JSON decoding. High-byte input adds
`raw_encoding: "ByteValuesU0000To00FF"`. This is reversible byte storage, not a
text encoding claim. Decoded STRING values still use Unicode. Fixed-size chunks
avoid a second input-sized raw projection allocation. Strict report fields and
bytes stay unchanged; nondefault choices add explicit fields to
`solidcore.step.source.v1`. C++ JSON bytes are not a compatibility promise.

The C++ contract distinguishes declaration `1` from corrected `2;1` (its fixed
ISO 10303-21:1994/Cor1:1996 replacement 9.2.1 reference). An assumption retains the
original declaration/span and strict `UnsupportedImplementationLevel` diagnostic;
it neither rewrites source nor proves original SELECT conformance. On `2;1`, an
enabled alternative is `NotApplied` and strict decoding still applies. Lowercase
interpretation never spreads to HEADER, directive spelling or unrelated recovery.

Each interpreted token retains original token/first-strict-error spans, lowercase
digit count and completion flag. Failures preserve complete tokens and incomplete
consumed prefixes with the complete captured bytes/hash, without a partial graph.
After reference resolution the receipt transfers once to the immutable snapshot;
clearing outcome diagnostic notices cannot erase its assumption. Failed requests
retain their own immutable receipt.

## Downstream boundary and validation

Default semantic checkpoints are strict. They refuse any requested or owner-applied
legacy assumption as `InvalidRequest`, including enabled-but-unused options:
original `STEP_SEMANTICS::Check` requires explicit general interoperability. C++
throws an argument error; Rust returns the structured
`UnverifiedCarrierRequiresInteroperability` request diagnostic. Opaque DATA in a
declared STRING field retains `OpaqueDataTextUnestablished` and its original
unsupported classification/position. The explicit interoperability path may
retain it only after its exact field/ownership proof; text stays unavailable and
cannot manufacture an owner. An opaque HEADER filename remains metadata.

Independent synthetic Rust tests cover byte recovery, exact policy boundaries,
failure precedence, default report compatibility, immutable receipts, completed
and incomplete token evidence, downstream refusal and token/report N/N-1 limits.
The real CLI tests include explicit policies and conflicting arguments.
`scripts/step_carrier_compare.py` invokes actual C++ and Rust source CLIs on the
same public bytes and compares source graph/references, capture hash, stable
diagnostics/spans, policies, receipts and counters, plus independent exact report
boundaries. Existing strict source/type/unit/ownership/geometry comparison runners
remain regression gates. This proves source compatibility, not geometric truth or
native import. Captures/receipts stay local; no private models are required.

```powershell
python -B scripts/step_carrier_compare.py --cpp /path/to/step_inspect.exe --rust bin/debug/step-inspect.exe
```
