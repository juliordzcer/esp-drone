/**
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * version.c - build version information
 */
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "param.h"

// Reemplaza los placeholders con valores estáticos para el build
const char * V_SLOCAL_REVISION="d9db942-dirty";
const char * V_SREVISION="v5.0";
const char * V_STAG="esp-drone";
const char * V_BRANCH="main";
const char * V_PROFILE=P_NAME;
const bool V_MODIFIED=true;

/* Version recoverable from the ground */
const uint32_t V_REVISION_0=0x00000000;
const uint16_t V_REVISION_1=0x0000;

// La sección PARAM_GROUP no es código C estándar.
// Es un macro de la cadena de construcción.
// Si tu sistema de parámetros no usa un script para esto,
// necesitarías una función para registrarlos.
// Aquí se muestra cómo se vería si se ignorara el macro.
// Si tu cadena de construcción aún lo necesita, no modifiques estas líneas.

// PARAM_GROUP_START(version)
// PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, revision0, &V_REVISION_0)
// PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, revision1, &V_REVISION_1)
// PARAM_GROUP_STOP(version)