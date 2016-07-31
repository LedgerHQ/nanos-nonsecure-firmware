/*******************************************************************************
*   Ledger Blue - Non secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
********************************************************************************/

#ifdef HAVE_BAGL

#include "bagl.h"
#include <string.h>
#include <stdio.h>

/**
 Coordinate system for BAGL:
 ===========================

       0   X axis
      0 +----->
        |
Y axis  |   ##### 
        v  #######
           ##   ##
           #######
            #####
*/


// --------------------------------------------------------------------------------------
// Checks
// -------------------------------------------------------------------------------------- 

/*
#ifndef BAGL_COMPONENT_MAXCOUNT
#error BAGL_COMPONENT_MAXCOUNT not set
#endif // !BAGL_COMPONENT_MAXCOUNT
*/

#ifndef BAGL_WIDTH
#error BAGL_WIDTH not set
#endif // !BAGL_WIDTH

#ifndef BAGL_HEIGHT
#error BAGL_HEIGHT not set
#endif // !BAGL_HEIGHT

// --------------------------------------------------------------------------------------
// Definitions
// -------------------------------------------------------------------------------------- 

#define ICON_WIDTH 0
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))
#define U2BE(buf, off) ((((buf)[off]&0xFF)<<8) | ((buf)[off+1]&0xFF) )
#define U4BE(buf, off) ((U2BE(buf, off)<<16) | (U2BE(buf, off+2)&0xFFFF))

// --------------------------------------------------------------------------------------
// Variables
// -------------------------------------------------------------------------------------- 

//bagl_component_t bagl_components[BAGL_COMPONENT_MAXCOUNT];
unsigned int bagl_bgcolor;

extern bagl_font_t const * C_bagl_fonts [];
// handle the touch with release out of zone
unsigned int last_touched_not_released_component_idx;

const bagl_glyph_array_entry_t* G_glyph_array;
unsigned int                    G_glyph_count; 


// --------------------------------------------------------------------------------------
// Dummy HAL
// -------------------------------------------------------------------------------------- 


// --------------------------------------------------------------------------------------

/*
// to draw font faster as the pixel loop is at the lower layer (if not directly within transport)
__weak void bagl_hal_draw_bitmap2_within_rect(unsigned int color_0, unsigned int color_1, unsigned short x, unsigned short y, unsigned short width, unsigned short height, unsigned char* bitmap2, unsigned short bitmap_length) {

}

// --------------------------------------------------------------------------------------

__weak void bagl_hal_draw_rect(unsigned int color, unsigned short x, unsigned short y, unsigned short width, unsigned short height) {

}

// --------------------------------------------------------------------------------------

// angle_start 0 => trigonometric 0, range is [0:360[
__weak void bagl_hal_draw_circle(unsigned int color, unsigned short x_center, unsigned short y_center, unsigned short radius, unsigned short angle_start, unsigned short angle_stop) {

}
*/

// --------------------------------------------------------------------------------------
// API
// -------------------------------------------------------------------------------------- 

// --------------------------------------------------------------------------------------
void bagl_draw_bg(unsigned int color) {
  bagl_component_t c;
  memset(&c, 0, sizeof(c));
  c.type = BAGL_RECTANGLE;
  c.userid = BAGL_NONE;
  bagl_bgcolor = c.fgcolor = color;
  c.x = 0;
  c.y = 0;
  c.width = BAGL_WIDTH;
  c.height = BAGL_HEIGHT;
  c.fill = BAGL_FILL;
  // draw the rect
  bagl_draw_with_context(&c, NULL, 0, 0);
}

// --------------------------------------------------------------------------------------
// return the width of a text (first line only) for alignment processing
unsigned short bagl_compute_line_width(unsigned short font_id, unsigned short width, void* text, unsigned char text_length, unsigned char text_encoding) {
  unsigned short xx;

  font_id &= (BAGL_FONT_ID_MASK);

  // check valid font
  if (font_id >= BAGL_FONT_LAST || C_bagl_fonts[font_id] == NULL) {
    return 0;
  }

  // initialize first index
  xx = 0;

  //printf("display text: %s\n", text);

  unsigned char ch_kerning = C_bagl_fonts[font_id]->char_kerning;

  // depending on encoding
  while (text_length--) {
    unsigned int ch = 0;
    // TODO support other encoding than ascii ISO8859 Latin
    switch(text_encoding) {
      default:
      case BAGL_ENCODING_LATIN1:
        ch = *((unsigned char*)text);
        text = (void*)(((unsigned char*)text)+1);
        if (ch < C_bagl_fonts[font_id]->first_char || ch > C_bagl_fonts[font_id]->last_char) {
          //printf("invalid char");
          // can't proceed
          // but return up to the invalid char
          return xx;
        }
        ch -= C_bagl_fonts[font_id]->first_char;
        break;
    }

    // retrieve the char bitmap
    unsigned char ch_width = C_bagl_fonts[font_id]->characters[ch].char_width;

    // go to next line if needed
    if (width > 0 && xx + ch_width > width) {
      return xx;
    }

    // prepare for next char
    xx += ch_width + ch_kerning;
  }
  return xx;
}

// --------------------------------------------------------------------------------------
// draw char until a char fit before reaching width
// TODO support hyphenation ??
int bagl_draw_string(unsigned short font_id, unsigned int fgcolor, unsigned int bgcolor, int x, int y, unsigned int width, unsigned int height, void* text, unsigned int text_length, unsigned char text_encoding) {
  unsigned int xx;
  unsigned int colors[2];
  colors[0] = bgcolor;
  colors[1] = fgcolor;

  font_id &= (BAGL_FONT_ID_MASK);

  // check valid font
  if (font_id >= BAGL_FONT_LAST || C_bagl_fonts[font_id] == NULL) {
    return 0;
  }

  // always comparing this way, very optimized etc
  width += x;
  height += y;

  // initialize first index
  xx = x;

  //printf("display text: %s\n", text);

  unsigned char ch_height = C_bagl_fonts[font_id]->char_height;
  unsigned char ch_kerning = C_bagl_fonts[font_id]->char_kerning;

  // depending on encoding
  while (text_length--) {
    unsigned int ch = 0;
    // TODO support other encoding than ascii ISO8859 Latin
    switch(text_encoding) {
      default:
      case BAGL_ENCODING_LATIN1:
        ch = *((unsigned char*)text);
        text = (void*)(((unsigned char*)text)+1);
        if (ch < C_bagl_fonts[font_id]->first_char || ch > C_bagl_fonts[font_id]->last_char) {
          //printf("invalid char");
          // can't proceed
          if (ch == '\n' || ch == '\t') {
            y += ch_height; // no interleave

            // IGNORED for first line
            if (y + ch_height > height) {
              // we're writing half height of the last line ... probably better to put some dashes
              return (y<<16)|(xx&0xFFFF);
            }

            // newline starts back at first x offset
            xx = x;

            continue;
          }
          return (y<<16)|(xx&0xFFFF);
        }
        ch -= C_bagl_fonts[font_id]->first_char;
        break;
    }

    // retrieve the char bitmap
    unsigned char const * ch_bitmap = &C_bagl_fonts[font_id]->bitmap[C_bagl_fonts[font_id]->characters[ch].bitmap_offset];
    unsigned char ch_width = C_bagl_fonts[font_id]->characters[ch].char_width;

    // go to next line if needed
    if (xx + ch_width > width) {
      y += ch_height; // no interleave

      // IGNORED for first line
      if (y + ch_height > height) {
        // we're writing half height of the last line ... probably better to put some dashes
        return (y<<16)|(xx&0xFFFF);
      }

      // newline starts back at first x offset
      xx = x;
    }

    /* IGNORED for first line
    if (y + ch_height > height) {
        // we're writing half height of the last line ... probably better to put some dashes
        return;
    }
    */

    // chars are storred LSB to MSB in each char, packed chars. horizontal scan
    bagl_hal_draw_bitmap_within_rect(xx, y, ch_width, ch_height, 2, colors, 1, ch_bitmap, ch_width*ch_height); // note, last parameter is computable could be avoided
    // prepare for next char
    xx += ch_width + ch_kerning;
  }

  // return newest position, for upcoming printf
  return (y<<16)|(xx&0xFFFF);
}

// --------------------------------------------------------------------------------------

// draw round or circle. unaliased.
// if radiusint is !=0 then draw a circle of color outline, and colorint inside
void bagl_draw_circle_helper(unsigned int color, int x_center, int y_center, unsigned int radius, unsigned char octants, unsigned int radiusint, unsigned int colorint) {

/*
   128 ***** 32
      *     *
  64 *       * 16
    *         *    
    *         *
   4 *       * 1
      *     *
     8 ***** 2
*/

  unsigned int last_x;
  int x = radius;
  int y = 0;
  int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0
  int dradius = radius-radiusint;
  last_x = x;
  unsigned int drawint = (radiusint > 0 && dradius > 0 /*&& xint <= yint*/);

  while( y <= x )
  {
    if (octants & 1) { // 
      if (drawint) {
        bagl_hal_draw_rect(colorint, x_center,   y+y_center, x-(dradius-1), 1);
        bagl_hal_draw_rect(color, x_center+x-(dradius-1), y+y_center, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center,   y+y_center-1, x, 1);
      }
    }
    if (octants & 2) { // 
      if (drawint) {
        if (last_x != x) {
          bagl_hal_draw_rect(colorint, x_center,   x+y_center, y-(dradius-1), 1);
        }
        bagl_hal_draw_rect(color, x_center+y-(dradius-1), x+y_center, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center,   x+y_center-1, y, 1);
      }
    }
    if (octants & 4) { // 
      if (drawint) {
        bagl_hal_draw_rect(colorint, x_center-x, y+y_center, x-(dradius-1), 1);
        bagl_hal_draw_rect(color, x_center-x-(dradius-1), y+y_center, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center-x, y+y_center-1, x, 1);
      }
    }
    if (octants & 8) { // 
      if (drawint) {
        if (last_x != x) {
          bagl_hal_draw_rect(colorint, x_center-y, x+y_center, y-(dradius-1), 1);
        }
        bagl_hal_draw_rect(color, x_center-y-(dradius-1), x+y_center, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center-y, x+y_center-1, y, 1);
      }
    }
    if (octants & 16) { //
      if (drawint) {
        bagl_hal_draw_rect(colorint, x_center,   y_center-y, x-(dradius-1), 1);
        bagl_hal_draw_rect(color, x_center+x-(dradius-1), y_center-y, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center,   y_center-y, x, 1);
      }
    }
    if (octants & 32) { // 
      if (drawint) {
        if (last_x != x) {
          bagl_hal_draw_rect(colorint, x_center,   y_center-x, y-(dradius-1), 1);
        }
        bagl_hal_draw_rect(color, x_center+y-(dradius-1), y_center-x, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center,   y_center-x, y, 1);
      }
    }
    if (octants & 64) { // 
      if (drawint) {
        bagl_hal_draw_rect(colorint, x_center-x, y_center-y, x-(dradius-1), 1);
        bagl_hal_draw_rect(color, x_center-x-(dradius-1), y_center-y, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center-x, y_center-y, x, 1);
      }
    }
    if (octants & 128) { //
      if (drawint) {
        if (last_x != x) {
          bagl_hal_draw_rect(colorint, x_center-y, y_center-x, y-(dradius-1), 1);
        }
        bagl_hal_draw_rect(color, x_center-y-(dradius-1), y_center-x, dradius, 1);
      }
      else {
        bagl_hal_draw_rect(color, x_center-y, y_center-x, y, 1);
      }
    }

    last_x = x;
    y++;
    if (decisionOver2<=0)
    {
      decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
    }
    else
    {
      x--;
      decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
    }
  }
}

// --------------------------------------------------------------------------------------

void bagl_set_glyph_array(const bagl_glyph_array_entry_t* array, unsigned int count) {
  G_glyph_array = array;
  G_glyph_count = count;
}

// --------------------------------------------------------------------------------------

void bagl_draw_with_context(bagl_component_t* component, void* context, unsigned short context_length, unsigned char context_encoding) {
  //unsigned char comp_idx;
  unsigned int halignment=0;
  unsigned int valignment=0;
  int x,y;
  unsigned int baseline=0;
  unsigned int char_height=0;
  unsigned int strwidth = 0;

  // DESIGN NOTE: always consider drawing onto a bg color filled image. (done upon undraw)

  /*
  // check if userid already exist, if yes, reuse entry
  for (comp_idx=0; comp_idx < BAGL_COMPONENT_MAXCOUNT; comp_idx++) {
    if (bagl_components[comp_idx].userid == component->userid) {
      goto idx_ok;
    }
  }

  // find the first empty entry
  for (comp_idx=0; comp_idx < BAGL_COMPONENT_MAXCOUNT; comp_idx++) {
    if (bagl_components[comp_idx].userid == BAGL_NONE) {
      goto idx_ok;
    }
  }
  // no more space :(
  //BAGL_THROW(NO_SPACE);
  return;


idx_ok:
  */
  
  // strip the flags to match kinds
  unsigned int type = component->type&~(BAGL_TYPE_FLAGS_MASK);

  // compute alignment if text provided and requiring special alignment
  if (context && context_length && type != BAGL_ICON) {
    baseline = C_bagl_fonts[component->font_id&BAGL_FONT_ID_MASK]->baseline_height;
    char_height = C_bagl_fonts[component->font_id&BAGL_FONT_ID_MASK]->char_height;
    strwidth = bagl_compute_line_width(component->font_id, component->width, context, context_length, context_encoding);
    switch (component->font_id & BAGL_FONT_ALIGNMENT_HORIZONTAL_MASK ) {
      default:
      case BAGL_FONT_ALIGNMENT_LEFT:
        halignment = 0;
        break;
      case BAGL_FONT_ALIGNMENT_RIGHT:
        halignment = component->width - strwidth;
        break;
      case BAGL_FONT_ALIGNMENT_CENTER:
        // x   xalign      strwidth width
        // '     '            '     '
        //       ^
        // xalign = x+ (width/2) - (strwidth/2) => align -x
        halignment = component->width/2 - strwidth/2;
        break;
    }

    switch (component->font_id & BAGL_FONT_ALIGNMENT_VERTICAL_MASK ) {
      default:
      case BAGL_FONT_ALIGNMENT_TOP:
        valignment = 0;
        break;
      case BAGL_FONT_ALIGNMENT_BOTTOM:
        valignment = component->height - baseline;
        break;
      case BAGL_FONT_ALIGNMENT_MIDDLE:
        // y                 yalign           charheight        height
        // '                    '          v  '                 '
        //                           baseline
        // yalign = y+ (height/2) - (baseline/2) => align - y
        valignment = component->height/2 - baseline/2 - 1;
        break;
    }
  }

  // only check the type only, ignore the touchable flag
  switch(type) {
    case BAGL_RECTANGLE:
    case BAGL_BUTTON: // textbox + rectangle
    draw_round_rect: {
      unsigned int radius = component->radius;
      //if (radius > component->width/2 ||radius > component->height/2) 
      {
        radius = MIN(radius, MIN(component->width/2, component->height/2));
      }
      // draw the background
      /* shall not be needed
      if (component->fill == BAGL_FILL) {
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+radius,                  
                           component->y, 
                           component->width-2*radius, 
                           component->stroke); // top
      }
      */
      
      if (component->fill != BAGL_FILL) {

        // inner
        // centered top to bottom
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+radius,                  
                           component->y, 
                           component->width-2*radius, 
                           component->height);
        // left to center rect
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x,                                    
                           component->y+radius, 
                           radius, 
                           component->height-2*radius); 
        // center rect to right
        bagl_hal_draw_rect(component->bgcolor, 
                           component->x+component->width-radius-1, 
                           component->y+radius, 
                           radius, 
                           component->height-2*radius);

        // outline
        // 4 rectangles (with last pixel of each corner not set)
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+radius,                  
                           component->y, 
                           component->width-2*radius, 
                           component->stroke); // top
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+radius,                  
                           component->y+component->height-1, 
                           component->width-2*radius, 
                           component->stroke); // bottom
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x,                                    
                           component->y+radius, 
                           component->stroke, 
                           component->height-2*radius); // left
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->width-1, 
                           component->y+radius, 
                           component->stroke, 
                           component->height-2*radius); // right
      }
      else {
        // centered top to bottom
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+radius,                  
                           component->y, 
                           component->width-2*radius, 
                           component->height);
        // left to center rect
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x,                                    
                           component->y+radius, 
                           radius, 
                           component->height-2*radius); 

        // center rect to right
        bagl_hal_draw_rect(component->fgcolor, 
                           component->x+component->width-radius, 
                           component->y+radius, 
                           radius, 
                           component->height-2*radius);
      }

      // draw corners
      if (radius > 1) {
        unsigned int radiusint = 0;
        // carve round when not filling
        if (component->fill != BAGL_FILL && component->stroke < radius) {
          radiusint = radius-component->stroke;
        }
        bagl_draw_circle_helper(component->fgcolor, 
                                component->x+radius, 
                                component->y+radius, 
                                radius, 
                                BAGL_FILL_CIRCLE_PI2_PI, 
                                radiusint, 
                                component->bgcolor);
        bagl_draw_circle_helper(component->fgcolor, 
                                component->x+component->width-radius-component->stroke, 
                                component->y+radius, 
                                radius, 
                                BAGL_FILL_CIRCLE_0_PI2, 
                                radiusint, 
                                component->bgcolor);
        bagl_draw_circle_helper(component->fgcolor, 
                                component->x+radius, 
                                component->y+component->height-radius-component->stroke, 
                                radius, 
                                BAGL_FILL_CIRCLE_PI_3PI2, 
                                radiusint, 
                                component->bgcolor);
        bagl_draw_circle_helper(component->fgcolor, 
                                component->x+component->width-radius-component->stroke, 
                                component->y+component->height-radius-component->stroke, 
                                radius, 
                                BAGL_FILL_CIRCLE_3PI2_2PI, 
                                radiusint, 
                                component->bgcolor);
      }

      
      // 1 textarea (reframed)
      if (context && context_length) {
        bagl_draw_string(component->font_id,
                         // draw '1' pixels in bg when filled
                         (component->fill == BAGL_FILL) ? component->bgcolor:component->fgcolor, 
                         // draw '0' pixels in fg when filled
                         (component->fill == BAGL_FILL) ? component->fgcolor:component->bgcolor, 
                         component->x + halignment, 
                         component->y + valignment, 
                         component->width - halignment - (MAX(1,component->stroke)*2), 
                         component->height - valignment - (MAX(1,component->stroke)*2),
                         context,
                         context_length,
                         context_encoding);
      }

      // centered by default
      if ( component->icon_id ) {
        goto case_BAGL_ICON;
      }
      break;
    }

    case BAGL_LABELINE:       
    case BAGL_LABEL: 
      // draw background rect
      if (component->fill == BAGL_FILL) 
      {
        bagl_hal_draw_rect(component->bgcolor,  // bg here, but need some helping
                           component->x, 
                           component->y-(type==BAGL_LABELINE?(baseline):0), 
                           component->width, 
                           (type==BAGL_LABELINE?char_height:component->height)
                           );
      }
      if (context && context_length) {
        // debug centering
        //bagl_hal_draw_rect(component->fgcolor, component->x, component->y, 2, 2);
        bagl_draw_string(component->font_id,
                         component->fgcolor, 
                         component->bgcolor, 
                         component->x + halignment /*+ component->stroke*/, 
                         component->y + (type==BAGL_LABELINE?-(baseline):valignment) /*+ component->stroke*/, 
                         component->width /*- 2*component->stroke*/ - halignment, 
                         component->height /*- 2*component->stroke*/ - (type==BAGL_LABELINE?0/*-char_height*/:valignment),
                         context,
                         context_length,
                         context_encoding);
        // debug centering
        //bagl_hal_draw_rect(component->fgcolor, component->x+component->width-2, component->y+component->height-2, 2, 2);
      }
      /*
      if (component->stroke > 0) {
        goto outline_rect;
      }
      */
      break;

    case BAGL_LINE: // a line is just a flat rectangle :p
      component->fill = BAGL_FILL; // mandat !!
    //case BAGL_RECTANGLE:
      if(component->fill == BAGL_FILL && component->radius == 0) {
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, component->height);
      }
      else {
        outline_rect:
        goto draw_round_rect;
        /*
        // not filled, respect the stroke
        // left
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, MAX(1,component->stroke), component->height);
        // top
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, MAX(1,component->stroke));
        // right
        bagl_hal_draw_rect(component->fgcolor, component->x+component->width-MAX(1,component->stroke), component->y, MAX(1,component->stroke), component->height);
        // bottom
        bagl_hal_draw_rect(component->fgcolor, component->x, component->y+component->height-MAX(1,component->stroke), component->width, MAX(1,component->stroke));
        */
      }
      break;

    case BAGL_ICON: {

      bagl_glyph_array_entry_t* glyph_array;
      unsigned int glyph_count;

    case_BAGL_ICON:
      x = component->x;
      y = component->y;

      // icon data follows are in the context
      if (component->icon_id != 0) {
        // select the default or custom glyph array
        if (component->type == BAGL_ICON && context_encoding && G_glyph_array && G_glyph_count > 0) {
          glyph_array = G_glyph_array;
          glyph_count = G_glyph_count;
        }
        else {
          glyph_array = C_glyph_array;
          glyph_count = C_glyph_count;
        }

        if (component->icon_id >= glyph_count) {
          // glyph doesn't exist, avoid printing random stuff from the memory
          break;
        }

        // color accounted as bytes in the context length
        if (context_length) {
          if ((1<<glyph_array[component->icon_id].bits_per_pixel)*4 != context_length) {
            // invalid color count
            break;
          }
          context_length /= 4;
        }
        // use default colors
        if (!context_length || !context) {
          context_length = 1<<(glyph_array[component->icon_id].bits_per_pixel);
          context = glyph_array[component->icon_id].default_colors;
        }

        // center glyph in rect
        // draw the glyph from the bitmap using the context for colors
        bagl_hal_draw_bitmap_within_rect(x + (component->width / 2 - glyph_array[component->icon_id].width / 2), 
                                         y + (component->height / 2 - glyph_array[component->icon_id].height / 2), 
                                         glyph_array[component->icon_id].width, 
                                         glyph_array[component->icon_id].height, 
                                         context_length, 
                                         (unsigned int*)context, // Endianness remarkably ignored !
                                         glyph_array[component->icon_id].bits_per_pixel, 
                                         glyph_array[component->icon_id].bitmap, 
                                         glyph_array[component->icon_id].bits_per_pixel*(glyph_array[component->icon_id].width*glyph_array[component->icon_id].height));
      }
      else {
        // context: <bitperpixel> [color_count*4 bytes (LE encoding)] <icon bitmap (raw scan, LE)>

        unsigned int colors[4];
        unsigned int bpp = ((unsigned char*)context)[0];
        if (bpp <= 2) {
          unsigned int i=1<<bpp;
          while(i--) {
            colors[i] = U4BE((unsigned char*)context, 1+i*4);
          }
        }

        // draw the glyph from the bitmap using the context for colors
        bagl_hal_draw_bitmap_within_rect(x, 
                                         y, 
                                         component->width, 
                                         component->height, 
                                         1<<bpp,
                                         colors,
                                         bpp, 
                                         ((unsigned char*)context)+1+(1<<bpp)*4, 
                                         bpp*(component->width*component->height));
      }

      break;
    }
    
    case BAGL_CIRCLE:
      // draw the circle (all 8 octants)
      bagl_draw_circle_helper(component->fgcolor, component->x+component->radius, component->y+component->radius, component->radius, 0xFF, ((component->fill != BAGL_FILL && component->stroke < component->radius)?component->radius-component->stroke:0), component->bgcolor);
      break;

    //case BAGL_NONE:
      // performing, but not registering
      //bagl_hal_draw_rect(component->fgcolor, component->x, component->y, component->width, component->height);
      //return;
    case BAGL_NONE:
    default:
      return;
  }

  /*
  // remember drawn component for user action
  memcpy(&bagl_components[comp_idx], component, sizeof(bagl_component_t));  
  */
}
    
// --------------------------------------------------------------------------------------

void bagl_animate(bagl_animated_t* anim, unsigned int timestamp_ms, unsigned int interval_ms) {
  // nothing to be animated right now (or no horizontal scrolling speed defined)
  if ((anim->next_ms != 0 && anim->next_ms > timestamp_ms) || anim->c.width == 0 || anim->c.icon_id == 0 || (anim->current_x & 0xF0000000)==0x40000000) {
    return;
  }

  // when starting the animation, perform a pause on the left of the string
  if (anim->next_ms == 0) {
    anim->next_ms = timestamp_ms + (anim->c.stroke&0x7F)*100;
    anim->current_x = 0x0;
    anim->current_char_idx = 0;
  }


  unsigned int valignment=0;
  unsigned int baseline=0;
  unsigned int char_height=0;
  unsigned int totalwidth = 0;
  unsigned int remwidth = 0;
  unsigned int charwidth=0;
  unsigned int type = anim->c.type&~(BAGL_TYPE_FLAGS_MASK);

  // compute alignment if text provided and requiring special alignment
  if (anim->text && anim->text_length && (type == BAGL_LABELINE || type == BAGL_LABEL)) {
    unsigned int maxcharwidth = bagl_compute_line_width(anim->c.font_id, 0, "W", 1, anim->text_encoding)+1;
    baseline = C_bagl_fonts[anim->c.font_id&BAGL_FONT_ID_MASK]->baseline_height;
    char_height = C_bagl_fonts[anim->c.font_id&BAGL_FONT_ID_MASK]->char_height;
    
    totalwidth = bagl_compute_line_width(anim->c.font_id, 0, anim->text, anim->text_length, anim->text_encoding);

    // nothing to be animated here, text is already fully drawn in its text box
    if (totalwidth <= anim->c.width) {
      return;
    }
    if (anim->current_char_idx > anim->text_length) {
      anim->current_char_idx = 0;
    }

    remwidth = bagl_compute_line_width(anim->c.font_id, 0, anim->text+anim->current_char_idx, anim->text_length-anim->current_char_idx, anim->text_encoding);
    charwidth = bagl_compute_line_width(anim->c.font_id, 0, anim->text+anim->current_char_idx, 1, anim->text_encoding);

    switch (anim->c.font_id & BAGL_FONT_ALIGNMENT_VERTICAL_MASK ) {
      default:
      case BAGL_FONT_ALIGNMENT_TOP:
        valignment = 0;
        break;
      case BAGL_FONT_ALIGNMENT_BOTTOM:
        valignment = anim->c.height - baseline;
        break;
      case BAGL_FONT_ALIGNMENT_MIDDLE:
        // y                 yalign           charheight        height
        // '                    '          v  '                 '
        //                           baseline
        // yalign = y+ (height/2) - (baseline/2) => align - y
        valignment = anim->c.height/2 - baseline/2 - 1;
        break;
    }

    // consider the current char of the string has been displayed on the screen (or at least a part of it)
    // viewport         |    < width >   |
    // totalwidth  | a text that does not fit the viewport |
    // rem width        |xt that does not fit the viewport |           s
    // 
    unsigned int current_char_displayed_width = (anim->current_x & ~(0xF0000000)) - (totalwidth - remwidth);

    // draw a bg rectangle on the area before painting the animated value, to clearup glitches on both sides
    bagl_draw_string(anim->c.font_id,
	         anim->c.fgcolor, 
	         anim->c.bgcolor, 
	         anim->c.x - current_char_displayed_width, 
	         anim->c.y + (type==BAGL_LABELINE?-(baseline):valignment) /*+ component->stroke*/, 
	         anim->c.width + current_char_displayed_width + charwidth /*- 2*component->stroke*/, 
	         anim->c.height /*- 2*component->stroke*/ - (type==BAGL_LABELINE?0/*-char_height*/:valignment),
	         anim->text+anim->current_char_idx, anim->text_length-anim->current_char_idx, anim->text_encoding);

    // crop the viewport
    bagl_hal_draw_rect(anim->c.bgcolor,
                       anim->c.x-maxcharwidth,
                       anim->c.y + (type==BAGL_LABELINE?-(baseline):valignment),
                       maxcharwidth,
                       anim->c.height- (type==BAGL_LABELINE?0/*-char_height*/:valignment));
    bagl_hal_draw_rect(anim->c.bgcolor,
                       anim->c.x+anim->c.width,
                       anim->c.y + (type==BAGL_LABELINE?-(baseline):valignment),
                       maxcharwidth,
                       anim->c.height- (type==BAGL_LABELINE?0/*-char_height*/:valignment));

  // report on screen
    screen_update();
    unsigned int step_ms=interval_ms;
    unsigned int step_x=anim->c.icon_id * step_ms/1000;
    while(step_x == 0) {
      step_ms += interval_ms;
      step_x = anim->c.icon_id * step_ms / 1000;
    }

    switch (anim->current_x & 0xF0000000) {
      // left to right
      case 0:
        anim->next_ms += step_ms;
        if (current_char_displayed_width >= charwidth) {
          anim->current_char_idx++;
          // if text fits, then stop scrolling and wait a bit
          if (remwidth - current_char_displayed_width <= anim->c.width) {
            anim->current_x = (totalwidth-remwidth+current_char_displayed_width) | 0x10000000;
            break;
          }
        }
        anim->current_x += step_x;
        break;
        
      // pause after finished left to right
      case 0x10000000:
        anim->next_ms += (anim->c.stroke&0x7F)*100;
        anim->current_x = (totalwidth-remwidth+current_char_displayed_width) | 0x20000000;
        break;

      // right to left
      case 0x20000000:
        anim->next_ms += step_ms;
        if (current_char_displayed_width >= charwidth) {
          // we're displaying from the start
          if (remwidth >= totalwidth) {
            anim->current_x = 0x30000000;
            anim->current_char_idx = 0;
            break;
          }
          anim->current_char_idx--;
        }
        anim->current_x = ((anim->current_x & ~(0xF0000000)) - step_x) | 0x20000000;
        break;
        
      // pause after finished right to left
      case 0x30000000:
        anim->next_ms += (anim->c.stroke&0x7F)*100;
        anim->current_x = 0;
        // not going for another scroll anim
        if (anim->c.stroke & 0x80) {
          anim->current_x = 0x40000000;
        }
        break;
      case 0x40000000:
        // stalled, nothing to do
        break;
    }
  }
}

// --------------------------------------------------------------------------------------

void bagl_draw(bagl_component_t* component) {
  // component without text
  bagl_draw_with_context(component, NULL, 0, 0);
}

#endif // HAVE_BAGL
