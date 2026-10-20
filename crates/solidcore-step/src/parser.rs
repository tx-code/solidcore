// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Strict corrected Part 21 lexical, structural and reference inspection.
//!
//! Upstream: `source/STEP_SOURCE/src/StepSource.cpp` (`Parser`).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | `Parser::parse` | `Parser::Parse` |
//! | `Parser::value` | `Parser::ParseValue` |
//! | `Parser::resolve` | `Parser::Resolve` |
//!
//! Deviations: strict carrier only; errors return owned receipts instead of
//! exceptions. Value descent and reference collection are bounded by the checked
//! nesting limit (hard ceiling 128). No allocation is sized from an instance ID.

use crate::{
    Component, Diagnostic, Limits, Record, Reference, Snapshot, Span, Statistics, Status, Value,
    ValueKind,
};
use std::collections::BTreeMap;
use std::sync::Arc;

pub(super) struct Failure {
    pub status: Status,
    pub diagnostic: Diagnostic,
}
type Parsed<T> = Result<T, Box<Failure>>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Kind {
    Word,
    Integer,
    Real,
    String,
    Enumeration,
    Reference,
    Left,
    Right,
    Comma,
    Equal,
    Semicolon,
    Unset,
    Derived,
    End,
}

struct Token {
    kind: Kind,
    span: Span,
    text: String,
}

pub(super) struct Parser {
    bytes: Arc<[u8]>,
    limits: Limits,
    stats: Statistics,
    pos: usize,
    token: Option<Token>,
    last_end: usize,
    record_id: String,
    component: String,
    path: Vec<usize>,
}

impl Parser {
    pub(super) fn new(bytes: Arc<[u8]>, limits: Limits) -> Self {
        Self {
            bytes,
            limits,
            stats: Statistics::default(),
            pos: 0,
            token: None,
            last_end: 0,
            record_id: String::new(),
            component: String::new(),
            path: Vec::new(),
        }
    }

    pub(super) fn statistics(&self) -> Statistics {
        self.stats
    }

    fn error(&self, key: &str, message: &str, span: Span, status: Status) -> Box<Failure> {
        Box::new(Failure {
            status,
            diagnostic: Diagnostic {
                key: key.into(),
                stage: "D0".into(),
                message: message.into(),
                primary: span,
                related: Vec::new(),
                record_id: self.record_id.clone(),
                component: self.component.clone(),
                parameter_path: self.path.clone(),
                target_id: String::new(),
            },
        })
    }

    fn rejected(&self, key: &str, message: &str, span: Span) -> Box<Failure> {
        self.error(key, message, span, Status::Rejected)
    }
    fn unsupported(&self, key: &str, message: &str, span: Span) -> Box<Failure> {
        self.error(key, message, span, Status::Unsupported)
    }
    fn bound(&self, value: usize, limit: usize, key: &str, span: Span) -> Parsed<()> {
        if value > limit {
            Err(self.error(
                key,
                "Configured source budget exceeded",
                span,
                Status::ResourceLimit,
            ))
        } else {
            Ok(())
        }
    }
    fn byte(&self, pos: usize) -> Option<u8> {
        self.bytes.get(pos).copied()
    }
    fn here(&self) -> Option<u8> {
        self.byte(self.pos)
    }
    fn span(&self, begin: usize) -> Span {
        Span {
            begin,
            end: self.pos,
        }
    }
    fn text(&self, span: Span) -> String {
        self.bytes
            .get(span.begin..span.end)
            .map_or_else(String::new, |bytes| {
                String::from_utf8_lossy(bytes).into_owned()
            })
    }

    fn string_position(&self, mut pos: usize) -> usize {
        // STEP_SOURCE's corrected carrier contract ignores physical layout even
        // inside an escape/doubled delimiter. Original spans still include it.
        while matches!(self.byte(pos), Some(b'\r' | b'\n' | b'\t')) {
            pos += 1;
        }
        pos
    }

    fn string_char(&mut self, start: usize, directive: usize) -> Parsed<u8> {
        self.pos = self.string_position(self.pos);
        self.bound(
            self.pos - start,
            self.limits.string_bytes,
            "StringByteLimit",
            self.span(start),
        )?;
        let c = self.here().ok_or_else(|| {
            self.rejected(
                "InvalidStringEncoding",
                "Incomplete string control directive",
                self.span(directive),
            )
        })?;
        self.bound(
            self.pos - start + 1,
            self.limits.string_bytes,
            "StringByteLimit",
            Span {
                begin: start,
                end: self.pos + 1,
            },
        )?;
        self.pos += 1;
        Ok(c)
    }

    fn string_expected(&mut self, expected: u8, start: usize, directive: usize) -> Parsed<()> {
        if self.string_char(start, directive)? != expected {
            return Err(self.rejected(
                "InvalidStringEncoding",
                "Malformed string control directive",
                self.span(directive),
            ));
        }
        Ok(())
    }

    fn hex(&mut self, digits: usize, start: usize, directive: usize) -> Parsed<u32> {
        let mut value = 0;
        for _ in 0..digits {
            let c = self.string_char(start, directive)?;
            let digit = match c {
                b'0'..=b'9' => u32::from(c - b'0'),
                b'A'..=b'F' => u32::from(c - b'A' + 10),
                _ => {
                    return Err(self.rejected(
                        "InvalidStringEncoding",
                        "Expected uppercase hexadecimal digits",
                        self.span(directive),
                    ))
                }
            };
            value = value * 16 + digit;
        }
        Ok(value)
    }

    fn code_point(&self, output: &mut String, value: u32, span: Span) -> Parsed<()> {
        // X2 groups are Unicode scalars, not UTF-16 code units. In particular,
        // paired surrogate groups remain invalid and must not be repaired here.
        let c = char::from_u32(value).ok_or_else(|| {
            self.rejected(
                "InvalidStringCodePoint",
                "Invalid Unicode scalar value",
                span,
            )
        })?;
        output.push(c);
        Ok(())
    }

    fn directive(&mut self, output: &mut String, start: usize, directive: usize) -> Parsed<()> {
        match self.string_char(start, directive)? {
            b'\\' => output.push('\\'),
            b'N' | b'F' => {
                self.string_expected(b'\\', start, directive)?;
                return Err(self.unsupported(
                    "UnsupportedStringEncoding",
                    "Print control directives are outside this slice",
                    self.span(directive),
                ));
            }
            b'S' => {
                self.string_expected(b'\\', start, directive)?;
                let c = self.string_char(start, directive)?;
                if !(0x20..=0x7e).contains(&c) {
                    return Err(self.rejected(
                        "InvalidStringEncoding",
                        "PAGE requires graphic ASCII",
                        self.span(directive),
                    ));
                }
                self.code_point(output, u32::from(c) + 128, self.span(directive))?;
            }
            b'P' => {
                let alphabet = self.string_char(start, directive)?;
                self.string_expected(b'\\', start, directive)?;
                if !(b'A'..=b'I').contains(&alphabet) {
                    return Err(self.rejected(
                        "InvalidStringEncoding",
                        "Invalid ISO 8859 alphabet",
                        self.span(directive),
                    ));
                }
                if alphabet != b'A' {
                    return Err(self.unsupported(
                        "UnsupportedStringEncoding",
                        "Only ISO 8859-1 is supported",
                        self.span(directive),
                    ));
                }
            }
            b'X' => {
                let mode = self.string_char(start, directive)?;
                if mode == b'\\' {
                    let value = self.hex(2, start, directive)?;
                    self.code_point(output, value, self.span(directive))?;
                    return Ok(());
                }
                let digits = match mode {
                    b'2' => 4,
                    b'4' => 8,
                    _ => {
                        return Err(self.rejected(
                            "InvalidStringEncoding",
                            "Expected X, X2 or X4 directive",
                            self.span(directive),
                        ))
                    }
                };
                self.string_expected(b'\\', start, directive)?;
                let mut any = false;
                loop {
                    let group = self.string_position(self.pos);
                    if matches!(self.byte(group), None | Some(b'\\')) {
                        break;
                    }
                    let value = self.hex(digits, start, directive)?;
                    self.code_point(output, value, self.span(group))?;
                    any = true;
                }
                if !any {
                    return Err(self.rejected(
                        "InvalidStringEncoding",
                        "Extended directive requires a code point",
                        self.span(directive),
                    ));
                }
                for expected in b"\\X0\\" {
                    self.string_expected(*expected, start, directive)?;
                }
            }
            _ => {
                return Err(self.rejected(
                    "InvalidStringEncoding",
                    "Unknown string control directive",
                    self.span(directive),
                ))
            }
        }
        Ok(())
    }

    fn string(&mut self, start: usize) -> Parsed<Token> {
        let mut text = String::new();
        while self.pos < self.bytes.len() {
            if self.string_position(self.pos) == self.bytes.len() {
                self.bound(
                    self.bytes.len() - start,
                    self.limits.string_bytes,
                    "StringByteLimit",
                    Span {
                        begin: start,
                        end: self.bytes.len(),
                    },
                )?;
                break;
            }
            let c = self.string_char(start, start)?;
            if c == b'\'' {
                let next = self.string_position(self.pos);
                if self.byte(next) == Some(b'\'') {
                    self.string_char(start, start)?;
                } else {
                    self.stats.max_string_bytes = self.stats.max_string_bytes.max(self.pos - start);
                    return Ok(Token {
                        kind: Kind::String,
                        span: self.span(start),
                        text,
                    });
                }
            } else if c == b'\\' {
                self.directive(&mut text, start, self.pos - 1)?;
                continue;
            }
            text.push(char::from(c));
        }
        Err(self.rejected(
            "UnterminatedString",
            "String has no closing apostrophe",
            Span {
                begin: start,
                end: self.bytes.len(),
            },
        ))
    }

    fn lex(&mut self) -> Parsed<Token> {
        loop {
            match self.here() {
                Some(b' ' | b'\t' | b'\r' | b'\n') => self.pos += 1,
                Some(b'/') if self.byte(self.pos + 1) == Some(b'*') => {
                    let start = self.pos;
                    self.pos += 2;
                    while self.pos < self.bytes.len()
                        && !(self.here() == Some(b'*') && self.byte(self.pos + 1) == Some(b'/'))
                    {
                        self.pos += 1;
                    }
                    if self.pos == self.bytes.len() {
                        return Err(self.rejected(
                            "UnterminatedComment",
                            "Block comment has no closing delimiter",
                            self.span(start),
                        ));
                    }
                    self.pos += 2;
                }
                _ => break,
            }
        }
        let start = self.pos;
        let Some(c) = self.here() else {
            return Ok(Token {
                kind: Kind::End,
                span: self.span(start),
                text: String::new(),
            });
        };
        self.pos += 1;
        let kind = match c {
            b'(' => Kind::Left,
            b')' => Kind::Right,
            b',' => Kind::Comma,
            b'=' => Kind::Equal,
            b';' => Kind::Semicolon,
            b'$' => Kind::Unset,
            b'*' => Kind::Derived,
            b'\'' => return self.string(start),
            b'#' => {
                let digits = self.pos;
                while self.here().is_some_and(|c| c.is_ascii_digit()) {
                    self.pos += 1;
                }
                if self.pos == digits {
                    return Err(self.rejected(
                        "InvalidInstanceId",
                        "Instance ID requires decimal digits",
                        self.span(start),
                    ));
                }
                self.bound(
                    self.pos - digits,
                    self.limits.id_digits,
                    "IdDigitLimit",
                    self.span(start),
                )?;
                let text = self.text(Span {
                    begin: digits,
                    end: self.pos,
                });
                // Canonical identity is decimal text, never an array index or
                // machine integer. Leading-zero aliases must collide as IDs.
                let canonical = text.trim_start_matches('0');
                if canonical.is_empty() {
                    return Err(self.rejected(
                        "InvalidInstanceId",
                        "Instance ID must be positive",
                        self.span(start),
                    ));
                }
                return Ok(Token {
                    kind: Kind::Reference,
                    span: self.span(start),
                    text: canonical.into(),
                });
            }
            b'.' => {
                let word = self.pos;
                while self
                    .here()
                    .is_some_and(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == b'_')
                {
                    self.pos += 1;
                }
                // Upstream Upper includes underscore (EXPRESS identifiers).
                if self.pos == word
                    || !self.byte(word).is_some_and(upper)
                    || self.here() != Some(b'.')
                {
                    // Match the upstream consumed-byte span when a closing character exists.
                    if self.pos != word
                        && self.byte(word).is_some_and(upper)
                        && self.here().is_some()
                    {
                        self.pos += 1;
                    }
                    return Err(self.rejected(
                        "InvalidEnumeration",
                        "Expected uppercase enumeration between full stops",
                        self.span(start),
                    ));
                }
                let text = self.text(Span {
                    begin: word,
                    end: self.pos,
                });
                self.pos += 1;
                return Ok(Token {
                    kind: Kind::Enumeration,
                    span: self.span(start),
                    text,
                });
            }
            b'"' | b'!' | b'@' | b'\\' => {
                return Err(self.unsupported(
                    "UnsupportedToken",
                    "Binary, user-defined and value-instance forms are not supported",
                    self.span(start),
                ))
            }
            c if upper(c) => {
                while self
                    .here()
                    .is_some_and(|c| upper(c) || c.is_ascii_digit() || c == b'-')
                {
                    self.pos += 1;
                }
                Kind::Word
            }
            b'0'..=b'9' | b'+' | b'-' => {
                if !c.is_ascii_digit() && !self.here().is_some_and(|c| c.is_ascii_digit()) {
                    return Err(self.rejected(
                        "InvalidNumber",
                        "Sign must immediately precede digits",
                        self.span(start),
                    ));
                }
                while self.here().is_some_and(|c| c.is_ascii_digit()) {
                    self.pos += 1;
                }
                let mut kind = Kind::Integer;
                if self.here() == Some(b'.') {
                    kind = Kind::Real;
                    self.pos += 1;
                    while self.here().is_some_and(|c| c.is_ascii_digit()) {
                        self.pos += 1;
                    }
                    if self.here() == Some(b'E') {
                        self.pos += 1;
                        if matches!(self.here(), Some(b'+' | b'-')) {
                            self.pos += 1;
                        }
                        let exponent = self.pos;
                        while self.here().is_some_and(|c| c.is_ascii_digit()) {
                            self.pos += 1;
                        }
                        if self.pos == exponent {
                            return Err(self.rejected(
                                "InvalidNumber",
                                "Exponent requires digits",
                                self.span(start),
                            ));
                        }
                    }
                }
                kind
            }
            _ => {
                return Err(self.rejected(
                    "UnexpectedCharacter",
                    "Unsupported Part21 character",
                    self.span(start),
                ))
            }
        };
        Ok(Token {
            kind,
            span: self.span(start),
            text: self.text(self.span(start)),
        })
    }

    fn peek(&mut self) -> Parsed<&Token> {
        if self.token.is_none() {
            self.token = Some(self.lex()?);
        }
        self.token
            .as_ref()
            .ok_or_else(|| self.rejected("SyntaxError", "Missing token", self.span(self.pos)))
    }
    fn kind(&mut self) -> Parsed<Kind> {
        Ok(self.peek()?.kind)
    }
    fn take(&mut self) -> Parsed<Token> {
        self.peek()?;
        let token = self
            .token
            .take()
            .ok_or_else(|| self.rejected("SyntaxError", "Missing token", self.span(self.pos)))?;
        self.last_end = token.span.end;
        Ok(token)
    }
    fn expect(&mut self, kind: Kind) -> Parsed<Token> {
        if self.kind()? != kind {
            let span = self.peek()?.span;
            return Err(self.rejected(
                "SyntaxError",
                "Unexpected token in exchange structure",
                span,
            ));
        }
        self.take()
    }
    fn word(&mut self, word: &str) -> Parsed<()> {
        let token = self.peek()?;
        if token.kind != Kind::Word || token.text != word {
            let span = token.span;
            return Err(self.rejected("SyntaxError", &format!("Expected {word}"), span));
        }
        self.take()?;
        Ok(())
    }

    fn parameters(&mut self, depth: usize) -> Parsed<Vec<Value>> {
        self.expect(Kind::Left)?;
        let mut values = Vec::new();
        if self.kind()? != Kind::Right {
            loop {
                self.path.push(values.len() + 1);
                values.push(self.value(depth + 1)?);
                self.path.pop();
                if self.kind()? != Kind::Comma {
                    break;
                }
                self.take()?;
            }
        }
        self.expect(Kind::Right)?;
        Ok(values)
    }

    fn value(&mut self, depth: usize) -> Parsed<Value> {
        let span = self.peek()?.span;
        self.bound(depth, self.limits.nesting, "NestingLimit", span)?;
        self.stats.values += 1;
        self.bound(self.stats.values, self.limits.values, "ValueLimit", span)?;
        self.stats.nesting = self.stats.nesting.max(depth);
        let token = self.take()?;
        let mut value = Value {
            kind: ValueKind::Unset,
            span,
            text: token.text,
            children: Vec::new(),
        };
        value.kind = match token.kind {
            Kind::Integer => ValueKind::Integer,
            Kind::Real => ValueKind::Real,
            Kind::String => ValueKind::String,
            Kind::Enumeration => ValueKind::Enumeration,
            Kind::Unset => ValueKind::Unset,
            Kind::Derived => ValueKind::Derived,
            Kind::Reference => {
                self.stats.references += 1;
                self.bound(
                    self.stats.references,
                    self.limits.references,
                    "ReferenceLimit",
                    span,
                )?;
                ValueKind::Reference
            }
            Kind::Left => {
                // Return the already-consumed '(' to the single-token buffer.
                self.token = Some(Token {
                    kind: Kind::Left,
                    span,
                    text: String::new(),
                });
                value.text.clear();
                value.children = self.parameters(depth)?;
                value.span.end = self.last_end;
                ValueKind::Aggregate
            }
            Kind::Word if name(&value.text) => {
                value.children = self.parameters(depth)?;
                value.span.end = self.last_end;
                if value.children.len() != 1 {
                    return Err(self.rejected(
                        "SyntaxError",
                        "Typed parameters require exactly one value",
                        value.span,
                    ));
                }
                ValueKind::Typed
            }
            _ => return Err(self.rejected("SyntaxError", "Unexpected value token", span)),
        };
        Ok(value)
    }

    fn parse_component(&mut self) -> Parsed<Component> {
        let token = self.expect(Kind::Word)?;
        if !name(&token.text) {
            return Err(self.rejected("SyntaxError", "Invalid entity keyword", token.span));
        }
        self.component.clone_from(&token.text);
        let parameters = self.parameters(0)?;
        Ok(Component {
            name: token.text,
            span: Span {
                begin: token.span.begin,
                end: self.last_end,
            },
            parameters,
        })
    }

    fn check_header(&self, component: &Component) -> Parsed<()> {
        let p = &component.parameters;
        let string_list = |v: &Value| {
            v.kind == ValueKind::Aggregate
                && !v.children.is_empty()
                && v.children.iter().all(|v| v.kind == ValueKind::String)
        };
        let string = |v: &Value| v.kind == ValueKind::String;
        let valid = match component.name.as_str() {
            "FILE_DESCRIPTION" => {
                let valid = p.len() == 2
                    && p.first().is_some_and(string_list)
                    && p.get(1).is_some_and(string);
                if valid {
                    if let Some(level) = p.get(1).filter(|level| level.text != "2;1") {
                        return Err(self.unsupported(
                            "UnsupportedImplementationLevel",
                            "Only FILE_DESCRIPTION implementation level 2;1 is supported",
                            level.span,
                        ));
                    }
                }
                valid
            }
            "FILE_NAME" => {
                p.len() == 7
                    && p.get(2).is_some_and(string_list)
                    && p.get(3).is_some_and(string_list)
                    && [0, 1, 4, 5, 6]
                        .iter()
                        .all(|i| p.get(*i).is_some_and(string))
            }
            "FILE_SCHEMA" => p.len() == 1 && p.first().is_some_and(string_list),
            _ => false,
        };
        if valid {
            Ok(())
        } else {
            Err(self.rejected(
                "InvalidHeader",
                "Header parameter shape is invalid",
                component.span,
            ))
        }
    }

    fn record(&mut self, records: &[Record], index: &BTreeMap<String, usize>) -> Parsed<Record> {
        let id = self.take()?;
        self.record_id.clone_from(&id.text);
        self.component.clear();
        self.bound(
            records.len() + 1,
            self.limits.records,
            "RecordLimit",
            id.span,
        )?;
        if let Some(previous) = index.get(&id.text).and_then(|i| records.get(*i)) {
            let mut failure = self.rejected(
                "DuplicateInstanceId",
                "Two DATA occurrences have the same instance ID",
                id.span,
            );
            failure.diagnostic.related.push(previous.id_span);
            failure.diagnostic.target_id.clone_from(&id.text);
            return Err(failure);
        }
        self.expect(Kind::Equal)?;
        let complex = self.kind()? == Kind::Left;
        let mut components = Vec::new();
        if complex {
            self.take()?;
            loop {
                components.push(self.parse_component()?);
                if self.kind()? != Kind::Word {
                    break;
                }
            }
            self.expect(Kind::Right)?;
        } else {
            components.push(self.parse_component()?);
        }
        let end = self.expect(Kind::Semicolon)?.span.end;
        let span = Span {
            begin: id.span.begin,
            end,
        };
        self.bound(
            end - span.begin,
            self.limits.record_bytes,
            "RecordByteLimit",
            span,
        )?;
        self.stats.max_record_bytes = self.stats.max_record_bytes.max(end - span.begin);
        self.stats.records += 1;
        self.record_id.clear();
        self.component.clear();
        Ok(Record {
            id: id.text,
            id_span: id.span,
            span,
            complex,
            components,
        })
    }

    fn collect(
        &self,
        value: &Value,
        record: &Record,
        component: &Component,
        r: usize,
        c: usize,
        path: &mut Vec<usize>,
        depth: usize,
        source: &mut crate::SourceData,
    ) -> Parsed<()> {
        self.bound(depth, self.limits.nesting, "NestingLimit", value.span)?;
        if value.kind == ValueKind::Reference {
            let Some(target_record_index) = source.index.get(&value.text).copied() else {
                let mut failure = self.rejected(
                    "UnresolvedRequiredReference",
                    "DATA reference has no target in the complete file",
                    value.span,
                );
                failure.diagnostic.stage = "D1".into();
                failure.diagnostic.record_id.clone_from(&record.id);
                failure.diagnostic.component.clone_from(&component.name);
                failure.diagnostic.parameter_path.clone_from(path);
                failure.diagnostic.target_id.clone_from(&value.text);
                return Err(failure);
            };
            source
                .incoming
                .entry(value.text.clone())
                .or_default()
                .push(source.references.len());
            source.references.push(Reference {
                target_id: value.text.clone(),
                span: value.span,
                record_index: r,
                component_index: c,
                parameter_path: path.clone(),
                target_record_index,
            });
        }
        for (i, child) in value.children.iter().enumerate() {
            path.push(i + 1);
            self.collect(child, record, component, r, c, path, depth + 1, source)?;
            path.pop();
        }
        Ok(())
    }

    fn resolve(&self, source: &mut crate::SourceData) -> Parsed<()> {
        // Move the immutable records out temporarily to avoid cloning source
        // values or exposing any graph before every target has been resolved.
        let records = std::mem::take(&mut source.records);
        for (r, record) in records.iter().enumerate() {
            for (c, component) in record.components.iter().enumerate() {
                for (p, value) in component.parameters.iter().enumerate() {
                    self.collect(value, record, component, r, c, &mut vec![p + 1], 1, source)?;
                }
            }
        }
        source.records = records;
        Ok(())
    }

    pub(super) fn parse(&mut self) -> Parsed<Snapshot> {
        if self.bytes.starts_with(b"PK") || matches!(self.byte(0), Some(b'<' | 0x1f)) {
            return Err(self.unsupported(
                "UnsupportedCarrier",
                "Only self-contained Part21 text is supported",
                Span {
                    begin: 0,
                    end: self.bytes.len().min(4),
                },
            ));
        }
        for (i, byte) in self.bytes.iter().enumerate() {
            if *byte >= 127 || (*byte < 32 && !matches!(*byte, b'\t' | b'\r' | b'\n')) {
                return Err(self.unsupported(
                    "UnsupportedEncoding",
                    "This reader slice accepts ASCII bytes only",
                    Span {
                        begin: i,
                        end: i + 1,
                    },
                ));
            }
        }
        self.word("ISO-10303-21")?;
        self.expect(Kind::Semicolon)?;
        self.word("HEADER")?;
        self.expect(Kind::Semicolon)?;
        let mut header = Vec::new();
        for required in ["FILE_DESCRIPTION", "FILE_NAME", "FILE_SCHEMA"] {
            let component = self.parse_component()?;
            if component.name != required {
                return Err(self.rejected(
                    "InvalidHeader",
                    "Required header records must be present in order",
                    component.span,
                ));
            }
            let end = self.expect(Kind::Semicolon)?.span.end;
            self.bound(
                end - component.span.begin,
                self.limits.record_bytes,
                "RecordByteLimit",
                Span {
                    begin: component.span.begin,
                    end,
                },
            )?;
            self.stats.max_record_bytes =
                self.stats.max_record_bytes.max(end - component.span.begin);
            self.check_header(&component)?;
            header.push(component);
        }
        self.component.clear();
        if self.peek()?.text != "ENDSEC" {
            let span = self.peek()?.span;
            return Err(self.unsupported(
                "UnsupportedHeaderRecord",
                "Additional HEADER records are not supported",
                span,
            ));
        }
        self.word("ENDSEC")?;
        self.expect(Kind::Semicolon)?;
        if matches!(self.peek()?.text.as_str(), "ANCHOR" | "REFERENCE") {
            let span = self.peek()?.span;
            return Err(self.unsupported(
                "UnsupportedSection",
                "External reference/anchor sections are not supported",
                span,
            ));
        }
        self.word("DATA")?;
        if self.kind()? == Kind::Left {
            let span = self.peek()?.span;
            return Err(self.unsupported(
                "UnsupportedDataSection",
                "Parameterized DATA sections are not supported",
                span,
            ));
        }
        self.expect(Kind::Semicolon)?;
        let mut records = Vec::new();
        let mut index = BTreeMap::new();
        while self.kind()? == Kind::Reference {
            let record = self.record(&records, &index)?;
            index.insert(record.id.clone(), records.len());
            records.push(record);
        }
        self.record_id.clear();
        self.component.clear();
        self.word("ENDSEC")?;
        self.expect(Kind::Semicolon)?;
        if matches!(
            self.peek()?.text.as_str(),
            "DATA" | "SIGNATURE" | "ANCHOR" | "REFERENCE"
        ) {
            let span = self.peek()?.span;
            return Err(self.unsupported(
                "UnsupportedSection",
                "Only one DATA section and no extensions are supported",
                span,
            ));
        }
        self.word("END-ISO-10303-21")?;
        self.expect(Kind::Semicolon)?;
        if self.kind()? != Kind::End {
            let span = self.peek()?.span;
            return Err(self.rejected(
                "TrailingContent",
                "Content after the end marker is not accepted",
                span,
            ));
        }
        let mut lines = vec![0];
        let mut pos = 0;
        while let Some(c) = self.byte(pos) {
            pos += 1;
            if c == b'\r' || c == b'\n' {
                if c == b'\r' && self.byte(pos) == Some(b'\n') {
                    pos += 1;
                }
                lines.push(pos);
            }
        }
        let mut source = crate::SourceData {
            bytes: Arc::clone(&self.bytes),
            header,
            records,
            references: Vec::new(),
            index,
            incoming: BTreeMap::new(),
            lines,
        };
        self.resolve(&mut source)?;
        Ok(Snapshot {
            data: Arc::new(source),
        })
    }
}

fn upper(c: u8) -> bool {
    c.is_ascii_uppercase() || c == b'_'
}
fn name(text: &str) -> bool {
    text.as_bytes().first().copied().is_some_and(upper)
        && text.bytes().all(|c| upper(c) || c.is_ascii_digit())
}
