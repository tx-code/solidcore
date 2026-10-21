// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Bind locally installed UI fonts explicitly; egui 0.37 bundles none by default.
use crate::scene::Result;
use std::{io::Read, path::PathBuf, sync::Arc};

fn read_font(path: &PathBuf) -> Option<Vec<u8>> {
    const MAX_BYTES: u64 = 32 * 1024 * 1024;
    let file = std::fs::File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.take(MAX_BYTES + 1).read_to_end(&mut bytes).ok()?;
    if bytes.is_empty() || u64::try_from(bytes.len()).ok()? > MAX_BYTES {
        None
    } else {
        Some(bytes)
    }
}

pub(crate) fn definitions() -> Result<egui::FontDefinitions> {
    // Use fonts installed with the operating system for local display only.
    // No system font is copied into the repository or redistributed. Enabling
    // egui/default_fonts would introduce OFL/Ubuntu-font licensed assets beyond
    // this repository's current dependency-license selection.
    let mut candidates = Vec::new();
    if let Some(directory) = std::env::var_os("WINDIR") {
        let directory = PathBuf::from(directory).join("Fonts");
        candidates.extend([directory.join("segoeui.ttf"), directory.join("arial.ttf")]);
    }
    candidates.extend([
        PathBuf::from("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        PathBuf::from("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"),
        PathBuf::from("/System/Library/Fonts/SFNS.ttf"),
        PathBuf::from("/System/Library/Fonts/Supplemental/Arial.ttf"),
    ]);
    let primary = candidates.iter().find_map(read_font).ok_or(
        "No installed UI font found (Segoe UI, DejaVu Sans, Liberation Sans or Arial required)",
    )?;
    let mut fonts = egui::FontDefinitions::empty();
    fonts.font_data.insert(
        "system-ui".into(),
        Arc::new(egui::FontData::from_owned(primary)),
    );
    let mut family = vec!["system-ui".into()];
    if let Some(directory) = std::env::var_os("WINDIR") {
        if let Some(data) = read_font(&PathBuf::from(directory).join("Fonts/msyh.ttc")) {
            fonts.font_data.insert(
                "system-cjk".into(),
                Arc::new(egui::FontData::from_owned(data)),
            );
            family.push("system-cjk".into());
        }
    }
    fonts
        .families
        .insert(egui::FontFamily::Proportional, family.clone());
    fonts.families.insert(egui::FontFamily::Monospace, family);
    Ok(fonts)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn installed_ui_font_has_real_ascii_glyphs_and_variable_widths() {
        let ctx = egui::Context::default();
        ctx.set_fonts(definitions().unwrap());
        let mut valid = false;
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            ui.label("solidcore Native STEP 0123456789");
            valid = ui.ctx().fonts_mut(|fonts| {
                let font = egui::FontId::proportional(14.);
                fonts.has_glyphs(&font, "solidcore Native STEP 0123456789")
                    && fonts.glyph_width(&font, 'W') > fonts.glyph_width(&font, 'i')
            });
        });
        // This headless glyph check intentionally has no GPU painter.
        output.textures_delta.clear();
        assert!(
            valid,
            "HUD must have actual ASCII glyphs, not the synthetic tofu fallback"
        );
    }
}
