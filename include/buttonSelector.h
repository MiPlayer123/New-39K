#ifndef __SELECTOR_H
#define __SELECTOR_H

extern int   autonomousSelection;

struct _button {
    int    xpos;
    int    ypos;
    int    width;
    int    height;
    bool   state;
    vex::color offColor;
    vex::color onColor;
    const char *label;
};

extern _button buttons[];

void displayButtonControls( int index, bool pressed );

void initButtons();

void userTouchCallbackPressed();

void userTouchCallbackReleased();

int findButton(  int16_t xpos, int16_t ypos );

#endif