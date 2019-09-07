#ifndef MATRIX_CONFIG_H_
#define MATRIX_CONFIG_H_

#if defined(KEYBOARD_CSCS64)
#define THIS_DEVICE_ROWS 5
#define MATRIX_ROW_PINS { PIN7, PIN8, PIN9, PIN10, PIN11 }
#define THIS_DEVICE_COLS 7
#define MATRIX_COL_PINS { PIN20, PIN19, PIN18, PIN17, PIN16, PIN15, PIN14 }

#define LAYOUT( \
  L00, L01, L02, L03, L04, L05,           R01, R02, R03, R04, R05, R06, \
  L10, L11, L12, L13, L14, L15, L16, R10, R11, R12, R13, R14, R15, R16, \
  L20, L21, L22, L23, L24, L25, L26, R20, R21, R22, R23, R24, R25, R26, \
  L30, L31, L32, L33, L34, L35, L36, R30, R31, R32, R33, R34, R35, R36, \
  L40,      L42, L43, L44, L45,           R41, R42, R43, R44,      R46  \
  ) \
  { \
    { L00, L01,   L02, L03, L04, L05, KC_NO }, \
    { L10, L11,   L12, L13, L14, L15, L16 }, \
    { L20, L21,   L22, L23, L24, L25, L26 }, \
    { L30, L31,   L32, L33, L34, L35, L36 }, \
    { L40, KC_NO, L42, L43, L44, L45, KC_NO }, \
    { R06, R05,   R04, R03, R02, R01, KC_NO }, \
    { R16, R15,   R14, R13, R12, R11, R10 }, \
    { R26, R25,   R24, R23, R22, R21, R20 }, \
    { R36, R35,   R34, R33, R32, R31, R30 }, \
    { R46, KC_NO, R44, R43, R42, R41, KC_NO }, \
  }

// Used to create a keymap using only KC_ prefixed keys
#define LAYOUT_kc( \
  L00, L01, L02, L03, L04, L05,           R01, R02, R03, R04, R05, R06, \
  L10, L11, L12, L13, L14, L15, L16, R10, R11, R12, R13, R14, R15, R16, \
  L20, L21, L22, L23, L24, L25, L26, R20, R21, R22, R23, R24, R25, R26, \
  L30, L31, L32, L33, L34, L35, L36, R30, R31, R32, R33, R34, R35, R36, \
  L40,      L42, L43, L44, L45,           R41, R42, R43, R44,      R46  \
  )									  \
  LAYOUT( \
    KC_##L00, KC_##L01, KC_##L02, KC_##L03, KC_##L04, KC_##L05,                     KC_##R01, KC_##R02, KC_##R03, KC_##R04, KC_##R05, KC_##R06, \
    KC_##L10, KC_##L11, KC_##L12, KC_##L13, KC_##L14, KC_##L15, KC_##L16, KC_##R10, KC_##R11, KC_##R12, KC_##R13, KC_##R14, KC_##R15, KC_##R16, \
    KC_##L20, KC_##L21, KC_##L22, KC_##L23, KC_##L24, KC_##L25, KC_##L26, KC_##R20, KC_##R21, KC_##R22, KC_##R23, KC_##R24, KC_##R25, KC_##R26, \
    KC_##L30, KC_##L31, KC_##L32, KC_##L33, KC_##L34, KC_##L35, KC_##L36, KC_##R30, KC_##R31, KC_##R32, KC_##R33, KC_##R34, KC_##R35, KC_##R36, \
    KC_##L40,           KC_##L42, KC_##L43, KC_##L44, KC_##L45,                     KC_##R41, KC_##R42, KC_##R43, KC_##R44,           KC_##R46 \
  )


#elif defined(KEYBOARD_CSCS52)
#define THIS_DEVICE_ROWS 4
#define MATRIX_ROW_PINS { PIN8, PIN9, PIN10, PIN11 }
#define THIS_DEVICE_COLS 7
#define MATRIX_COL_PINS { PIN20, PIN19, PIN18, PIN17, PIN16, PIN15, PIN14 }

#define LAYOUT( \
  L10, L11, L12, L13, L14, L15, L16, R10, R11, R12, R13, R14, R15, R16, \
  L20, L21, L22, L23, L24, L25, L26, R20, R21, R22, R23, R24, R25, R26, \
  L30, L31, L32, L33, L34, L35, L36, R30, R31, R32, R33, R34, R35, R36, \
  L40,      L42, L43, L44, L45,           R41, R42, R43, R44,      R46  \
  ) \
  { \
    { L10, L11,   L12, L13, L14, L15, L16 }, \
    { L20, L21,   L22, L23, L24, L25, L26 }, \
    { L30, L31,   L32, L33, L34, L35, L36 }, \
    { L40, KC_NO, L42, L43, L44, L45, KC_NO }, \
    { R16, R15,   R14, R13, R12, R11, R10 }, \
    { R26, R25,   R24, R23, R22, R21, R20 }, \
    { R36, R35,   R34, R33, R32, R31, R30 }, \
    { R46, KC_NO, R44, R43, R42, R41, KC_NO }, \
  }

// Used to create a keymap using only KC_ prefixed keys
#define LAYOUT_kc( \
  L10, L11, L12, L13, L14, L15, L16, R10, R11, R12, R13, R14, R15, R16, \
  L20, L21, L22, L23, L24, L25, L26, R20, R21, R22, R23, R24, R25, R26, \
  L30, L31, L32, L33, L34, L35, L36, R30, R31, R32, R33, R34, R35, R36, \
  L40,      L42, L43, L44, L45,           R41, R42, R43, R44,      R46  \
  )									  \
  LAYOUT( \
    KC_##L10, KC_##L11, KC_##L12, KC_##L13, KC_##L14, KC_##L15, KC_##L16, KC_##R10, KC_##R11, KC_##R12, KC_##R13, KC_##R14, KC_##R15, KC_##R16, \
    KC_##L20, KC_##L21, KC_##L22, KC_##L23, KC_##L24, KC_##L25, KC_##L26, KC_##R20, KC_##R21, KC_##R22, KC_##R23, KC_##R24, KC_##R25, KC_##R26, \
    KC_##L30, KC_##L31, KC_##L32, KC_##L33, KC_##L34, KC_##L35, KC_##L36, KC_##R30, KC_##R31, KC_##R32, KC_##R33, KC_##R34, KC_##R35, KC_##R36, \
    KC_##L40,           KC_##L42, KC_##L43, KC_##L44, KC_##L45,                     KC_##R41, KC_##R42, KC_##R43, KC_##R44,           KC_##R46 \
  )

#else
//#error please define keyboard type
#undef THIS_DEVICE_ROWS
#undef MATRIX_ROW_PINS
#undef THIS_DEVICE_COLS
#undef MATRIX_COL_PINS
#undef LAYOUT
#undef LAYOUT_kc
#endif

#endif /* MATRIX_CONFIG_H_ */
