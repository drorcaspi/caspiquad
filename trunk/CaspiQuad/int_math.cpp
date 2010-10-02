#if 0
//=============================================================================
//
// Integer Math Utilities
//
//=============================================================================

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2010 Dror Caspi.  All rights reserved.

  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
-----------------------------------------------------------------------------*/

#include "quad.h"
#include "int_math.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

// None


//=============================================================================
//
// Static Variables
//
//=============================================================================

// None


//=============================================================================
//
// Public Variables
//
//=============================================================================

// None


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================= sqrt32() ======================================
//
// Integer square root
// Based on "Integer Square Root" article by Jack W. Crenshaw,
// http://www.embedded.com/98/9802fe2.htm

uint16_t sqrt32(uint32_t a)

{
  uint32_t rem  = 0;
  uint32_t root = 0;
  uint8_t  i;

  for (i = 0; i < 16; i++)
  {
    rem = ((rem << 2) + (a >> 30));
    a <<= 2;
    root = (root << 1) + 1;
    
    if (root <= rem)
    {
      rem -= root;
      root++;
    }
    
    else
      root--;
  }

  return (uint16_t)(root >> 1);
}

#endif
