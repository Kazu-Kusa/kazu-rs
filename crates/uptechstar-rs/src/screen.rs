//! LCD screen, LED, font, and color modules.
//!
//! Port of `pyuptech/modules/screen.py`.

use log::info;

use crate::ffi;

// ── ScreenDirection ──────────────────────────────────────────

/// Screen orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScreenDirection {
    /// 64×128
    Vertical = 1,
    /// 128×64
    Horizontal = 2,
}

impl ScreenDirection {
    pub fn width(&self) -> i32 {
        match self {
            ScreenDirection::Vertical => 64,
            ScreenDirection::Horizontal => 128,
        }
    }

    pub fn height(&self) -> i32 {
        match self {
            ScreenDirection::Vertical => 128,
            ScreenDirection::Horizontal => 64,
        }
    }
}

impl From<i32> for ScreenDirection {
    fn from(v: i32) -> Self {
        match v {
            1 => ScreenDirection::Vertical,
            _ => ScreenDirection::Horizontal,
        }
    }
}

// ── FontSize ─────────────────────────────────────────────────

/// Available font sizes for the LCD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum FontSize {
    Font4x6 = 0,
    Font5x8 = 1,
    Font5x12 = 2,
    Font6x8 = 3,
    Font6x10 = 4,
    Font7x12 = 5,
    Font8x8 = 6,
    Font8x12 = 7,
    Font8x14 = 8,
    Font10x16 = 9,
    Font12x16 = 10,
    Font12x20 = 11,
    Font16x26 = 12,
    Font22x36 = 13,
    Font24x40 = 14,
}

impl FontSize {
    /// Row height in pixels.
    pub fn row_height(&self) -> i32 {
        match self {
            FontSize::Font4x6 => 6,
            FontSize::Font5x8 => 8,
            FontSize::Font5x12 => 12,
            FontSize::Font6x8 => 8,
            FontSize::Font6x10 => 10,
            FontSize::Font7x12 => 12,
            FontSize::Font8x8 => 8,
            FontSize::Font8x12 => 12,
            FontSize::Font8x14 => 14,
            FontSize::Font10x16 => 16,
            FontSize::Font12x16 => 16,
            FontSize::Font12x20 => 20,
            FontSize::Font16x26 => 26,
            FontSize::Font22x36 => 36,
            FontSize::Font24x40 => 40,
        }
    }

    /// Column width in pixels.
    pub fn column_width(&self) -> i32 {
        match self {
            FontSize::Font4x6 => 4,
            FontSize::Font5x8 => 5,
            FontSize::Font5x12 => 5,
            FontSize::Font6x8 => 6,
            FontSize::Font6x10 => 6,
            FontSize::Font7x12 => 7,
            FontSize::Font8x8 => 8,
            FontSize::Font8x12 => 8,
            FontSize::Font8x14 => 8,
            FontSize::Font10x16 => 10,
            FontSize::Font12x16 => 12,
            FontSize::Font12x20 => 12,
            FontSize::Font16x26 => 16,
            FontSize::Font22x36 => 22,
            FontSize::Font24x40 => 24,
        }
    }
}

// ── Color ────────────────────────────────────────────────────

/// 24-bit RGB color packed as `(R << 16) | (G << 8) | B`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Color(pub i32);

impl Color {
    /// Create a new color from R, G, B components (0–255 each).
    pub const fn new(r: i32, g: i32, b: i32) -> Self {
        Color((r << 16) | (g << 8) | b)
    }

    pub const WHITE: Color = Color::new(255, 255, 255);
    pub const GRAY: Color = Color::new(128, 128, 128);
    pub const BLACK: Color = Color::new(0, 0, 0);

    pub const RED: Color = Color::new(255, 0, 0);
    pub const GREEN: Color = Color::new(0, 255, 0);
    pub const BLUE: Color = Color::new(0, 0, 255);

    pub const B_RED: Color = Color::new(255, 0, 128);
    pub const G_RED: Color = Color::new(255, 128, 0);

    pub const G_BLUE: Color = Color::new(0, 128, 255);
    pub const R_BLUE: Color = Color::new(128, 0, 255);

    pub const R_GREEN: Color = Color::new(128, 255, 0);
    pub const B_GREEN: Color = Color::new(0, 255, 128);

    pub const YELLOW: Color = Color::new(255, 255, 0);
    pub const MAGENTA: Color = Color::new(255, 0, 255);
    pub const CYAN: Color = Color::new(0, 255, 255);

    pub const ORANGE: Color = Color::new(128, 128, 0);
    pub const PURPLE: Color = Color::new(128, 0, 128);
    pub const BLUEGREEN: Color = Color::new(0, 128, 128);

    pub const DARKBLUE: Color = Color::new(0, 0, 139);
    pub const DARKGREEN: Color = Color::new(0, 139, 0);
    pub const DARKRED: Color = Color::new(139, 0, 0);
}

impl From<Color> for i32 {
    fn from(c: Color) -> i32 {
        c.0
    }
}

impl From<i32> for Color {
    fn from(v: i32) -> Self {
        Color(v)
    }
}

// ── Screen ───────────────────────────────────────────────────

/// LCD screen controller with chainable API.
///
/// Each method returns `Self` for builder-pattern chaining.
pub struct Screen {
    screen_dir: Option<ScreenDirection>,
    font_size: FontSize,
}

impl Screen {
    /// Create a new `Screen`.
    ///
    /// If `screen_dir` is provided, the screen is opened, filled black, and refreshed.
    pub fn new(screen_dir: Option<i32>) -> Self {
        let mut s = Self {
            screen_dir: None,
            font_size: FontSize::Font12x20,
        };
        if let Some(dir) = screen_dir {
            s.open(dir).fill_screen(Color::BLACK).refresh();
        }
        s
    }

    /// Open the LCD and set the display direction.
    ///
    /// `direction`: 1 = vertical (64×128), 2 = horizontal (128×64).
    pub fn open(&mut self, direction: i32) -> &mut Self {
        info!("Open LCD with direction: {}", direction);
        ffi::lcd_open(direction);
        self.screen_dir = Some(ScreenDirection::from(direction));
        self
    }

    /// Close the LCD.
    pub fn close(&mut self) -> &mut Self {
        info!("Closing LCD");
        ffi::lcd_close();
        self
    }

    /// Refresh the screen (flush display buffer).
    pub fn refresh(&mut self) -> &mut Self {
        ffi::lcd_refresh();
        self
    }

    /// Set the current font size.
    pub fn set_font_size(&mut self, font_size: FontSize) -> &mut Self {
        self.font_size = font_size;
        ffi::lcd_set_font(font_size as i32);
        self
    }

    /// Set the foreground (text) color.
    pub fn set_fore_color(&mut self, color: impl Into<Color>) -> &mut Self {
        ffi::ug_set_forecolor(color.into().0);
        self
    }

    /// Set the background color.
    pub fn set_back_color(&mut self, color: impl Into<Color>) -> &mut Self {
        ffi::ug_set_backcolor(color.into().0);
        self
    }

    /// Set an LED color by index (0 or 1).
    pub fn set_led_color(&mut self, index: i32, color: impl Into<Color>) -> &mut Self {
        ffi::adc_led_set(index, color.into().0);
        self
    }

    /// Set LED 0 color.
    pub fn set_led_0(&mut self, color: impl Into<Color>) -> &mut Self {
        self.set_led_color(0, color)
    }

    /// Set LED 1 color.
    pub fn set_led_1(&mut self, color: impl Into<Color>) -> &mut Self {
        self.set_led_color(1, color)
    }

    /// Set both LEDs to the same color.
    pub fn set_all_leds_same(&mut self, color: impl Into<Color>) -> &mut Self {
        let c = color.into().0;
        ffi::adc_led_set(0, c);
        ffi::adc_led_set(1, c);
        self
    }

    /// Set the two LEDs to different colors.
    pub fn set_all_leds_single(
        &mut self,
        first: impl Into<Color>,
        second: impl Into<Color>,
    ) -> &mut Self {
        ffi::adc_led_set(0, first.into().0);
        ffi::adc_led_set(1, second.into().0);
        self
    }

    /// Turn both LEDs off.
    pub fn set_all_leds_off(&mut self) -> &mut Self {
        ffi::adc_led_set(0, 0);
        ffi::adc_led_set(1, 0);
        self
    }

    /// Fill the entire screen with a color.
    pub fn fill_screen(&mut self, color: impl Into<Color>) -> &mut Self {
        ffi::ug_fill_screen(color.into().0);
        self
    }

    /// Draw a string at the given pixel coordinates.
    pub fn put_string(&mut self, x: i32, y: i32, s: &str) -> &mut Self {
        ffi::ug_put_string(x, y, s);
        self
    }

    /// Print a string at the top-left corner.
    pub fn print(&mut self, s: &str) -> &mut Self {
        ffi::ug_put_string(0, 0, s);
        self
    }

    /// Fill a rectangular frame.
    pub fn fill_frame(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_fill_frame(x1, y1, x2, y2, color.into().0);
        self
    }

    /// Fill a rounded rectangular frame.
    pub fn fill_round_frame(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        r: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_fill_round_frame(x1, y1, x2, y2, r, color.into().0);
        self
    }

    /// Fill a circle.
    pub fn fill_circle(
        &mut self,
        x0: i32,
        y0: i32,
        r: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_fill_circle(x0, y0, r, color.into().0);
        self
    }

    /// Draw a mesh pattern within a rectangle.
    pub fn draw_mesh(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_mesh(x1, y1, x2, y2, color.into().0);
        self
    }

    /// Draw an empty rectangular frame.
    pub fn draw_frame(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_frame(x1, y1, x2, y2, color.into().0);
        self
    }

    /// Draw an empty rounded rectangular frame.
    pub fn draw_round_frame(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        r: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_round_frame(x1, y1, x2, y2, r, color.into().0);
        self
    }

    /// Draw a single pixel.
    pub fn draw_pixel(&mut self, x0: i32, y0: i32, color: impl Into<Color>) -> &mut Self {
        ffi::ug_draw_pixel(x0, y0, color.into().0);
        self
    }

    /// Draw an empty circle.
    pub fn draw_circle(
        &mut self,
        x0: i32,
        y0: i32,
        r: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_circle(x0, y0, r, color.into().0);
        self
    }

    /// Draw an arc.
    pub fn draw_arc(
        &mut self,
        x0: i32,
        y0: i32,
        r: i32,
        s: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_arc(x0, y0, r, s, color.into().0);
        self
    }

    /// Draw a line between two points.
    pub fn draw_line(
        &mut self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: impl Into<Color>,
    ) -> &mut Self {
        ffi::ug_draw_line(x1, y1, x2, y2, color.into().0);
        self
    }
}

impl Default for Screen {
    fn default() -> Self {
        Self::new(None)
    }
}
