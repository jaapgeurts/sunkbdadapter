/*
  Keymap for Sun Keyboard to ASCII
  Fields are:
  Sun_KeyCode   ASCII_Code  Description
 */

#define CMD_RESET         0x01
#define CMD_DISABLE_CLICK 0x0B
#define CMD_ENABLE_CLICK  0x0A
#define CMD_ENABLE_BELL   0x02
#define CMD_DISABLE_BELL  0x03

static const int sun_to_ascii[128] = {
 /* 0 */  	0,
 /* 1 */	0x78,	/* Stop */
 /* 2 */	0,	/* Volume_Decr */
 /* 3 */	0,	/* Again */
 /* 4 */	0,	/* Volume_Incr */
 /* 5 */	194,	/* F1 */
 /* 6 */	195,	/* F2 */
 /* 7 */	203,	/* F10 */
 /* 8 */	196,	/* F3 */
 /* 9 */	204,	/* F11 */
 /* 10 */	197,	/* F4 */
 /* 11 */	205,	/* F12 */
 /* 12 */	198,	/* F5 */
 /* 13 */       0,
 /* 14 */	199,	/* F6 */
 /* 15 */	0,
 /* 16 */	200,	/* F7 */
 /* 17 */	201,	/* F8 */
 /* 18 */	202,	/* F9 */
 /* 19 */	130,	/* Alt_L */
 /* 20 */	218,	/* T5_Up */
 /* 21 */	0,	/* Pause/Break */
 /* 22 */	0,	/* Print_Screen */
 /* 23 */	0,	/* Scroll_Lock */
 /* 24 */	216,	/* T5_Left */
 /* 25 */	0x76,	/* Props */
 /* 26 */	0,	/* Undo */
 /* 27 */	217,	/* T5_Down */
 /* 28 */	215,	/* T5_Right */
 /* 29 */	177,	/* Esc */
 /* 30 */	49,	/* 1 */
 /* 31 */	50,	/* 2 */
 /* 32 */	51,	/* 3 */
 /* 33 */	52,	/* 4 */
 /* 34 */	53,	/* 5 */
 /* 35 */	54,	/* 6 */
 /* 36 */	55,	/* 7 */
 /* 37 */	56,	/* 8 */
 /* 38 */	57,	/* 9 */
 /* 39 */	48,	/* 0 */
 /* 40 */	45,	/* minus */
 /* 41 */	61,	/* equal */
 /* 42 */	126,	/* grave/tilde */
 /* 43 */	178,	/* BackSpace */
 /* 44 */	209,	/* T5_Insert */
 /* 45 */	0,	/* Mute */
 /* 46 */	47,	/* R5/KP_Div */
 /* 47 */	42,	/* R6/KP_Mult */
 /* 48 */	0,	/* Power */
 /* 49 */	0,	/* Front */
 /* 50 */	46,	/* KP_./KP_Delete */
 /* 51 */	0,	/* Copy */
 /* 52 */	210,	/* T5_Home */
 /* 53 */	179,	/* Tab */
 /* 54 */	113,	/* Q */
 /* 55 */	119,	/* W */
 /* 56 */	101,	/* E */
 /* 57 */	114,	/* R */
 /* 58 */	116,	/* T */
 /* 59 */	121,	/* Y */
 /* 60 */	117,	/* U */
 /* 61 */	105,	/* I */
 /* 62 */	111,	/* O */
 /* 63 */	112,	/* P */
 /* 64 */	91,	/* [ */
 /* 65 */	93,	/* ] */
 /* 66 */	212,	/* Delete */
 /* 67 */       0,
 /* 68 */	55,	/* KP_7/Home */
 /* 69 */	56,	/* KP_8/Up */
 /* 70 */	57,	/* KP_9/PgUp */
 /* 71 */	45,	/* KP_Minus */
 /* 72 */	0,	/* Open */
 /* 73 */	0,	/* Paste */
 /* 74 */	213,	/* T5_End */
 /* 75 */ 0,
 /* 76 */	128,	/* Ctrl_L */
 /* 77 */	97,	/* A */
 /* 78 */	115,	/* S */
 /* 79 */	100,	/* D */
 /* 80 */	102,	/* F */
 /* 81 */	103,	/* G */
 /* 82 */	104,	/* H */
 /* 83 */	106,	/* J */
 /* 84 */	107,	/* K */
 /* 85 */	108,	/* L */
 /* 86 */	59,	/* ; */
 /* 87 */	39,	/* apostr. */
 /* 88 */	92,	/* backslash */
 /* 89 */	176,	/* Return */
 /* 90 */	176,	/* KP_Enter */
 /* 91 */	52,	/* KP_4/Left */
 /* 92 */	53,	/* KP_5/Center */
 /* 93 */	54,	/* KP_6/Right */
 /* 94 */	48,	/* KP_0/KP_Insert */
 /* 95 */	0,	/* Find */
 /* 96 */	211,	/* T5_PgUp */
 /* 97 */	0,	/* Cut */
 /* 98 */	0,	/* Num_Lock */
 /* 99 */	129,	/* Shift_L */
 /* 100 */	122,	/* Z */
 /* 101 */	120,	/* X */
 /* 102 */	99,	/* C */
 /* 103 */	118,	/* V */
 /* 104 */	98,	/* B */
 /* 105 */	110,	/* N */
 /* 106 */	109,	/* M */
 /* 107 */	44,	/* , */
 /* 108 */	46,	/* . */
 /* 109 */	47,	/* / */
 /* 110 */	133,	/* Shift_R */
 /* 111 */	0,
 /* 112 */	49,	/* KP_1/End */
 /* 113 */	50,	/* KP_2/Down */
 /* 114 */	51,	/* KP_3/PgDn */
 /* 115 */	0,
 /* 116 */	0,
 /* 117 */	0,
 /* 118 */	0,	/* Help */
 /* 119 */	193,	/* CapsLock */
 /* 120 */	131,	/* Meta_L */
 /* 121 */	32,	/* SpaceBar */
 /* 122 */	135,	/* Meta_R */
 /* 123 */	214,	/* T5_PgDn */
 /* 124 */	0,
 /* 125 */	43,	/* KP_Add */
 /* 126 */ 	0,
 /* 127 */	0
};
