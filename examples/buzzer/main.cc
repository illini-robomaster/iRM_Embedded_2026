/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "main.h"

#include "bsp_buzzer.h"
#include "tim.h"

using Note = bsp::BuzzerNote;

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
    {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}};
// uncomment to play the full song
static bsp::BuzzerNoteDelayed War_Cant_of_Mars[] = {
    //    {Note::So5M, 400},   {Note::So5M, 200},  {Note::So5M, 200},
    //    {Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
    //    {Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
    //    {Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
    //    {Note::Re2H, 200},   {Note::Do1H, 400},  {Note::Do1H, 400},
    //    {Note::Si7M, 200},   {Note::La6M, 400},  {Note::La6M, 200},
    //    {Note::La6M, 400},   {Note::Si7M, 200},  {Note::Do1H, 400},
    //    {Note::Si7M, 200},   {Note::Do1H, 400},  {Note::La6M, 200},
    //    {Note::So5M, 400},   {Note::La6M, 200},  {Note::So5M, 400},
    //    {Note::Mi3M, 200},   {Note::So5M, 800},  {Note::So5M, 400},
    //    {Note::So5M, 200},   {Note::So5M, 400},  {Note::So5M, 200},
    //    {Note::So5M, 400},   {Note::Fa4M, 200},  {Note::Mi3M, 400},
    //    {Note::So5M, 200},   {Note::Do1H, 400},  {Note::Re2H, 200},
    //    {Note::Mi3H, 400},   {Note::Mi3H, 200},  {Note::Mi3H, 400},
    //    {Note::Re2H, 200},   {Note::Do1H, 800},  {Note::Do1H, 800},
    //    {Note::Re2H, 800},   {Note::Re2H, 800},  {Note::Do1H, 800},
    //    {Note::Si7M, 800},   {Note::Do1H, 1600}, {Note::Silent, 400},
    {Note::Silent, 400},
    {Note::So5M, 800},
    {Note::Fa4M, 400},
    {Note::Mi3M, 400},
    {Note::So5M, 200},
    {Note::Do1H, 400},
    {Note::Re2H, 200},
    {Note::Mi3H, 1200},
    {Note::Do1H, 800},
    //    {Note::Silent, 400}, {Note::La6M, 800},   {Note::Si7M, 400},
    //    {Note::Do1H, 400},   {Note::Si7M, 200},   {Note::Do1H, 400},
    //    {Note::La6M, 200},   {Note::So5M, 1600},  {Note::Mi3M, 800},
    //    {Note::Silent, 400}, {Note::So5M, 800},   {Note::Fa4M, 400},
    //    {Note::Mi3M, 400},   {Note::So5M, 200},   {Note::Do1H, 400},
    //    {Note::Re2H, 200},   {Note::Mi3H, 1600},  {Note::Do1H, 800},
    //    {Note::Do1H, 800},   {Note::Re2H, 800},   {Note::Re2H, 800},
    //    {Note::Do1H, 800},   {Note::Si7M, 800},   {Note::Do1H, 1600},
    {Note::Silent, 0},
    {Note::Finish, 0},
};

static bsp::BuzzerNoteDelayed Imperial_March[] = {
        {Note::NOTE_AS4, 250},
        {Note::NOTE_AS4, 250},
        {Note::NOTE_AS4, 250},//1
        {Note::NOTE_F5, 1000},
        {Note::NOTE_C6, 1000},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F6, 1000},
        {Note::NOTE_C6, 500},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F6, 1000},
        {Note::NOTE_C6, 500},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_G5, 1000},
        {Note::NOTE_C5, 250},
        {Note::NOTE_C5, 250},
        {Note::NOTE_C5, 250},
        {Note::NOTE_F5, 1000},
        {Note::NOTE_C6, 1000},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F6, 1000},
        {Note::NOTE_C6, 500},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F6, 1000},
        {Note::NOTE_C6, 500},//8
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_G5, 1000},
        {Note::NOTE_C5, 450},
        {Note::NOTE_C5, 16},
        {Note::NOTE_D5, 900},
        {Note::NOTE_D5, 250},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F5, 250},
        {Note::NOTE_F5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 500},
        {Note::NOTE_D5, 250},
        {Note::NOTE_E5, 500},
        {Note::NOTE_C5, 450},
        {Note::NOTE_C5, 16},
        {Note::NOTE_D5, 900},
        {Note::NOTE_D5, 250},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F5, 250},
        {Note::NOTE_C6, 450},
        {Note::NOTE_G5, 16},
        {Note::NOTE_G5, 2},
        {Note::Silent, 250},
        {Note::NOTE_C5, 250},//13
        {Note::NOTE_D5, 900},
        {Note::NOTE_D5, 250},
        {Note::NOTE_AS5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_F5, 250},
        {Note::NOTE_F5, 250},
        {Note::NOTE_G5, 250},
        {Note::NOTE_A5, 250},
        {Note::NOTE_G5, 500},
        {Note::NOTE_D5, 250},
        {Note::NOTE_E5, 500},
        {Note::NOTE_C6, 450},
        {Note::NOTE_C6, 16},
        {Note::NOTE_F6, 500},
        {Note::NOTE_DS6, 250},
        {Note::NOTE_CS6, 500},
        {Note::NOTE_C6, 250},
        {Note::NOTE_AS5, 500},
        {Note::NOTE_GS5, 250},
        {Note::NOTE_G5, 500},
        {Note::NOTE_F5, 250},
        {Note::NOTE_C6, 2000},
        {Note::Silent, 0},
        {Note::Finish, 0},
};

void RM_RTOS_Init(void) {
  bsp::Buzzer buzzer(&htim4, 3, 1000000);
  UNUSED(Mario);
  UNUSED(War_Cant_of_Mars);
//  buzzer.SingSong(Mario);
//  buzzer.SingSong(War_Cant_of_Mars);
  buzzer.SingSong(Imperial_March);
}




