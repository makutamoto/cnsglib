/**
* @file colors.h
* \~english @brief Defines colors for Command Prompt.
* \~japanese @brief コマンドプロンプトで使える色を定義。
*/

#ifndef COLOR_H
#define COLOR_H

typedef enum {
  BLACK,  DARK_RED, DARK_GREEN, DARK_YELLOW,
  DARK_BLUE,  DARK_MAGENTA, DARK_CYAN,  DARK_GRAY,
  GRAY, RED,  GREEN,  YELLOW,
  BLUE, MAGENTA,  CYAN, WHITE,
  NULL_COLOR,
} Color;

#endif
