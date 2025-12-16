/*
 Copyright (C) 2025  <uliuc@gmx.net >
 based on code by Tara K. https://github.com/TaraHoleInIt/MinivMacArduino
 based on OSGLUSDL.c
 TODO: remove unused SDL code
 TODO: sound suppport (e.g. I2S)

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License as published by the
 Free Software Foundation; either version 3 of the License, or (at your
 option) any later version.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 for more details.

 For the complete text of the GNU General Public License see
 http://www.gnu.org/licenses/.
 
*/

#include "CNFGRAPI.h"
#include "SYSDEPNS.h"
#include "ENDIANAC.h"
#include "MYOSGLUE.h"
#include "STRCONST.h"

#include "ESP32API.h"

#include "esp_log.h"

const char* TAG = "OSGLUEESP32";

/* --- some simple utilities --- */

GLOBALOSGLUPROC MyMoveBytes(anyp srcPtr, anyp destPtr, si5b byteCount)
{
	(void) memcpy((char *)destPtr, (char *)srcPtr, byteCount);
}

#ifdef _WIN32
#define MyPathSep '\\'
#else
#define MyPathSep '/'
#endif

LOCALFUNC tMacErr ChildPath(char *x, char *y, char **r)
{
        tMacErr err = mnvm_miscErr;
        int nx = strlen(x);
        int ny = strlen(y);
        {
                if ((nx > 0) && (MyPathSep == x[nx - 1])) {
                        --nx;
                }
                {
                        int nr = nx + 1 + ny;
                        char *p = ESP32API_malloc(nr + 1);
                        if (p != NULL) {
                                char *p2 = p;
                                (void) memcpy(p2, x, nx);
                                p2 += nx;
                                *p2++ = MyPathSep;
                                (void) memcpy(p2, y, ny);
                                p2 += ny;
                                *p2 = 0;
                                *r = p;
                                err = mnvm_noErr;
                        }
                }
        }

        return err;
}


/* --- control mode and internationalization --- */

#define NeedCell2PlainAsciiMap 1

#define dbglog_OSGInit (0 && dbglog_HAVE)

#include "INTLCHAR.h"

LOCALVAR char *d_arg = NULL;

LOCALPROC MyMayFree(char *p)
{
	if (NULL != p) {
		ESP32API_free(p);
	}
}

/* --- sending debugging info to file --- */

#if dbglog_HAVE

LOCALFUNC blnr dbglog_open0(void)
{
	return trueblnr;
}

LOCALPROC dbglog_write0(char *s, uimr L)
{
    ESP_LOGI(TAG, "In Debuglog");
	ESP_LOGI(TAG, "%.*s", (int)L, s);
}

LOCALPROC dbglog_close0(void)
{
}

#endif

/* --- information about the environment --- */

#define WantColorTransValid 0

#include "COMOSGLU.h"
#include "PBUFSTDC.h"
#include "CONTROLM.h"

/* --- text translation --- */

LOCALPROC NativeStrFromCStr(char *r, char *s)
{
	ui3b ps[ClStrMaxLength];
	int i;
	int L;

	ClStrFromSubstCStr(&L, ps, s);

	for (i = 0; i < L; ++i) {
		r[i] = Cell2PlainAsciiMap[ps[i]];
	}

	r[L] = 0;
}

/* --- drives --- */

/*
	OSGLUxxx common:
	define NotAfileRef to some value that is different
	from any valid open file reference.
*/
#define NotAfileRef NULL

#define MyFilePtr ESP32File
#define MySeek ESP32API_seek
#define MySeekSet ESP32_Seek_Set
#define MySeekCur ESP32_Seek_Cur
#define MySeekEnd ESP32_Seek_End
#define MyFileRead ESP32API_read
#define MyFileWrite ESP32API_write
#define MyFileTell ESP32API_tell
#define MyFileClose ESP32API_close
#define MyFileOpen ESP32API_open
#define MyFileEof ESP32API_eof

LOCALVAR MyFilePtr Drives[NumDrives]; /* open disk image files */

LOCALPROC InitDrives(void)
{
	/*
		This isn't really needed, Drives[i] and DriveNames[i]
		need not have valid values when not vSonyIsInserted[i].
	*/
	tDrive i;

	for (i = 0; i < NumDrives; ++i) {
		Drives[i] = NotAfileRef;
	}
}

GLOBALOSGLUFUNC tMacErr vSonyTransfer(blnr IsWrite, ui3p Buffer,
	tDrive Drive_No, ui5r Sony_Start, ui5r Sony_Count,
	ui5r *Sony_ActCount)
{
	/*
		OSGLUxxx common:
		return 0 if it succeeds, nonzero (a
		Macintosh style error code, but -1
		will do) on failure.
	*/
	tMacErr err = mnvm_miscErr;
	MyFilePtr refnum = Drives[Drive_No];
	ui5r NewSony_Count = 0;

	if (MySeek(refnum, Sony_Start, MySeekSet) >= 0) {
		if (IsWrite) {
			NewSony_Count = MyFileWrite(Buffer, 1, Sony_Count, refnum);
		} else {
			NewSony_Count = MyFileRead(Buffer, 1, Sony_Count, refnum);
		}

		if (NewSony_Count == Sony_Count) {
			err = mnvm_noErr;
		}
	}

	if (nullpr != Sony_ActCount) {
		*Sony_ActCount = NewSony_Count;
	}

	return err; /*& figure out what really to return &*/
}

GLOBALOSGLUFUNC tMacErr vSonyGetSize(tDrive Drive_No, ui5r *Sony_Count)
{
	/*
		OSGLUxxx common:
		set Sony_Count to the size of disk image number Drive_No.

		return 0 if it succeeds, nonzero (a
		Macintosh style error code, but -1
		will do) on failure.
	*/
	tMacErr err = mnvm_miscErr;
	MyFilePtr refnum = Drives[Drive_No];
	long v;

	if (MySeek(refnum, 0, MySeekEnd) >= 0) {
		v = MyFileTell(refnum);
		if (v >= 0) {
			*Sony_Count = v;
			err = mnvm_noErr;
		}
	}

	return err; /*& figure out what really to return &*/
}

LOCALFUNC tMacErr vSonyEject0(tDrive Drive_No, blnr deleteit)
{
	/*
		OSGLUxxx common:
		close disk image number Drive_No.

		return 0 if it succeeds, nonzero (a
		Macintosh style error code, but -1
		will do) on failure.
	*/
	MyFilePtr refnum = Drives[Drive_No];

	DiskEjectedNotify(Drive_No);

	MyFileClose(refnum);
	Drives[Drive_No] = NotAfileRef; /* not really needed */

	return mnvm_noErr;
}

GLOBALOSGLUFUNC tMacErr vSonyEject(tDrive Drive_No)
{
	return vSonyEject0(Drive_No, falseblnr);
}

LOCALPROC UnInitDrives(void)
{
	tDrive i;

	for (i = 0; i < NumDrives; ++i) {
		if (vSonyIsInserted(i)) {
			(void) vSonyEject(i);
		}
	}
}

LOCALFUNC blnr Sony_Insert0(MyFilePtr refnum, blnr locked,
	char *drivepath)
{
	/*
		OSGLUxxx common:
		Given reference to open file, mount it as a disk image file.
		if "locked", then mount it as a locked disk.
	*/

	tDrive Drive_No;
	blnr IsOk = falseblnr;

	if (! FirstFreeDisk(&Drive_No)) {
		MacMsg(kStrTooManyImagesTitle, kStrTooManyImagesMessage,
			falseblnr);
	} else {
		/* printf("Sony_Insert0 %d\n", (int)Drive_No); */

		{
			Drives[Drive_No] = refnum;
			DiskInsertNotify(Drive_No, locked);

			IsOk = trueblnr;
		}
	}

	if (! IsOk) {
		MyFileClose(refnum);
	}

	return IsOk;
}

LOCALFUNC blnr Sony_Insert1(char *drivepath, blnr silentfail)
{
	blnr locked = falseblnr;
	/* printf("Sony_Insert1 %s\n", drivepath); */
	MyFilePtr refnum = MyFileOpen(drivepath, "rb+");
	if (NULL == refnum) {
		locked = trueblnr;
		refnum = MyFileOpen(drivepath, "rb");
	}
	if (NULL == refnum) {
		if (! silentfail) {
			MacMsg(kStrOpenFailTitle, kStrOpenFailMessage, falseblnr);
		}
	} else {
		return Sony_Insert0(refnum, locked, drivepath);
	}
	return falseblnr;
}

LOCALFUNC tMacErr LoadMacRomFrom(char *path)
{
	tMacErr err;
	MyFilePtr ROM_File;
	int File_Size;

    ESP_LOGI(TAG, "%s", path);

	ROM_File = MyFileOpen(path, "rb");
	if (NULL == ROM_File) {
		err = mnvm_fnfErr;
	} else {
		File_Size = MyFileRead(ROM, 1, kROM_Size, ROM_File);
		if (File_Size != kROM_Size) {
#ifdef MyFileEof
			if (MyFileEof(ROM_File))
#else
			if (File_Size > 0)
#endif
			{
				MacMsgOverride(kStrShortROMTitle,
					kStrShortROMMessage);
				err = mnvm_eofErr;
			} else {
				MacMsgOverride(kStrNoReadROMTitle,
					kStrNoReadROMMessage);
				err = mnvm_miscErr;
			}
		} else {
			err = ROM_IsValid();
		}
		MyFileClose(ROM_File);
	}
	
	ESP_LOGI(TAG, "Error: %d", err);

	return err;
}

LOCALFUNC blnr Sony_Insert2(char *s)
{
	char *d = d_arg;
	blnr IsOk = falseblnr;

	if (NULL == d) {
		IsOk = Sony_Insert1(s, trueblnr);
	} else
	{
		char *t = NULL;

		if (mnvm_noErr == ChildPath(d, s, &t)) {
			IsOk = Sony_Insert1(t, trueblnr);
		}

		MyMayFree(t);
	}

	return IsOk;
}

LOCALFUNC blnr Sony_InsertIth(int i)
{
	blnr v;

	if ((i > 9) || ! FirstFreeDisk(nullpr)) {
		v = falseblnr;
	} else {
		char s[] = "disk?.dsk";

		s[4] = '0' + i;

		v = Sony_Insert2(s);
	}

	return v;
}

LOCALFUNC blnr LoadInitialImages(void)
{
	if (! AnyDiskInserted()) {
		int i;

		for (i = 1; Sony_InsertIth(i); ++i) {
			/* stop on first error (including file not found) */
		}
	}

	return trueblnr;
}

/* --- ROM --- */

LOCALVAR char *rom_path = NULL;

LOCALFUNC blnr LoadMacRom(void)
{
	tMacErr err;

	if ((NULL == rom_path)
		|| (mnvm_fnfErr == (err = LoadMacRomFrom(rom_path))))
	if (mnvm_fnfErr == (err = LoadMacRomFrom(RomFileName)))
	{
        ESP_LOGI(TAG, "could not load rom file");
	}

	return trueblnr; /* keep launching Mini vMac, regardless */
}

/* --- video out --- */

LOCALVAR blnr gBackgroundFlag = falseblnr;
LOCALVAR blnr gTrueBackgroundFlag = falseblnr;
LOCALVAR blnr CurSpeedStopped = falseblnr;

LOCALPROC HaveChangedScreenBuff(ui4r top, ui4r left, ui4r bottom, ui4r right) {
	ESP32API_ScreenChanged(top, left, bottom, right);
}

LOCALPROC MyDrawChangesAndClear(void)
{
	if (ScreenChangedBottom > ScreenChangedTop) {
		HaveChangedScreenBuff(ScreenChangedTop, ScreenChangedLeft,
			ScreenChangedBottom, ScreenChangedRight);
		ScreenClearChanges();
	}
}

GLOBALOSGLUPROC DoneWithDrawingForTick(void)
{
#if EnableFSMouseMotion
	if (HaveMouseMotion) {
		AutoScrollScreen();
	}
#endif
	MyDrawChangesAndClear();
}

/* --- mouse --- */

/* cursor hiding */

LOCALPROC ForceShowCursor(void)
{
}

/* cursor moving */

/*
	OSGLUxxx common:
	When "EnableFSMouseMotion" the platform
	specific code can get relative mouse
	motion, instead of absolute coordinates
	on the emulated screen. It should
	set HaveMouseMotion to true when
	it is doing this (normally when in
	full screen mode.)

	This can usually be implemented by
	hiding the platform specific cursor,
	and then keeping it within a box,
	moving the cursor back to the center whenever
	it leaves the box. This requires the
	ability to move the cursor (in MyMoveMouse).
*/

#ifndef HaveWorkingWarp
#define HaveWorkingWarp 1
#endif

#if EnableMoveMouse && HaveWorkingWarp
LOCALFUNC blnr MyMoveMouse(si4b h, si4b v)
{
	/*
		OSGLUxxx common:
		Move the cursor to the point h, v on the emulated screen.
		If detect that this fails return falseblnr,
			otherwise return trueblnr.
		(On some platforms it is possible to move the curser,
			but there is no way to detect failure.)
	*/

	return trueblnr;
}
#endif

/* cursor state */

LOCALPROC CheckMouseState(void)
{
	int MouseH = 0;
	int MouseV = 0;
	int dx = 0;
	int dy = 0;

	ESP32API_GetMouseDelta( &dx, &dy );

	MyMousePositionSetDelta( dx, dy );
	MyMouseButtonSet( ESP32API_GetMouseButton( ) );

	MouseH = CurMouseH;
	MouseV = CurMouseV;

	ESP32API_GiveEmulatedMouseToESP32( &MouseH, &MouseV );
}

/* --- keyboard input --- */

// keyboard mapping from hid to mac
LOCALFUNC ui3r HIDKeyCode2MacKeyCode(ui3r hid_code)
{
    ui3r v = MKC_None;

    switch (hid_code) {
        case 0x2A: v = MKC_BackSpace; break;      // HID Backspace
        case 0x2B: v = MKC_Tab; break;            // HID Tab
        case 0x28: v = MKC_Return; break;         // HID Enter
        case 0x39: v = MKC_formac_CapsLock; break; // HID Caps Lock
        case 0x29: v = MKC_formac_Escape; break;  // HID Escape
        case 0x2C: v = MKC_Space; break;          // HID Space
        
        case 0x04: v = MKC_A; break;
        case 0x05: v = MKC_B; break;
        case 0x06: v = MKC_C; break;
        case 0x07: v = MKC_D; break;
        case 0x08: v = MKC_E; break;
        case 0x09: v = MKC_F; break;
        case 0x0A: v = MKC_G; break;
        case 0x0B: v = MKC_H; break;
        case 0x0C: v = MKC_I; break;
        case 0x0D: v = MKC_J; break;
        case 0x0E: v = MKC_K; break;
        case 0x0F: v = MKC_L; break;
        case 0x10: v = MKC_M; break;
        case 0x11: v = MKC_N; break;
        case 0x12: v = MKC_O; break;
        case 0x13: v = MKC_P; break;
        case 0x14: v = MKC_Q; break;
        case 0x15: v = MKC_R; break;
        case 0x16: v = MKC_S; break;
        case 0x17: v = MKC_T; break;
        case 0x18: v = MKC_U; break;
        case 0x19: v = MKC_V; break;
        case 0x1A: v = MKC_W; break;
        case 0x1B: v = MKC_X; break;
        case 0x1C: v = MKC_Y; break;
        case 0x1D: v = MKC_Z; break;
            
        // numbers
        case 0x1E: v = MKC_1; break;
        case 0x1F: v = MKC_2; break;
        case 0x20: v = MKC_3; break;
        case 0x21: v = MKC_4; break;
        case 0x22: v = MKC_5; break;
        case 0x23: v = MKC_6; break;
        case 0x24: v = MKC_7; break;
        case 0x25: v = MKC_8; break;
        case 0x26: v = MKC_9; break;
        case 0x27: v = MKC_0; break;
    
        // special characters
        case 0x2D: v = MKC_Minus; break;
        case 0x2E: v = MKC_Equal; break;
        case 0x31: v = MKC_formac_BackSlash; break;
        case 0x2F: v = MKC_LeftBracket; break;
        case 0x30: v = MKC_RightBracket; break;
        case 0x33: v = MKC_SemiColon; break;
        //case 0x35: v = MKC_Quote; break;
        case 0x35: v = MKC_formac_Grave; break;
        case 0x36: v = MKC_Comma; break;
        case 0x37: v = MKC_Period; break;
        case 0x38: v = MKC_formac_Slash; break;
    
        // arrow keys
        case 0x52: v = MKC_Up; break;
        case 0x51: v = MKC_Down; break;
        case 0x4F: v = MKC_Right; break;
        case 0x50: v = MKC_Left; break;
    
        // F-Keys
        case 0x3A: v = MKC_formac_F1; break;
        case 0x3B: v = MKC_formac_F2; break;
        case 0x3C: v = MKC_formac_F3; break;
        case 0x3D: v = MKC_formac_F4; break;
        case 0x3E: v = MKC_formac_F5; break;
        case 0x3F: v = MKC_F6; break;
        case 0x40: v = MKC_F7; break;
        case 0x41: v = MKC_F8; break;
        case 0x42: v = MKC_F9; break;
        case 0x43: v = MKC_F10; break;
        case 0x44: v = MKC_F11; break;
        case 0x45: v = MKC_F12; break;
     
        // modifier keys
        case 0xE0: v = MKC_formac_Control; break;   // LCtrl
        case 0xE1: v = MKC_formac_Shift; break;     // LShift
        case 0xE2: v = MKC_formac_Option; break;    // LAlt
        case 0xE3: v = MKC_formac_Command; break;   // LMeta/Command
        case 0xE4: v = MKC_formac_RControl; break;
        case 0xE5: v = MKC_formac_RShift; break;
        case 0xE6: v = MKC_formac_ROption; break;
        case 0xE7: v = MKC_formac_RCommand; break;
    
        // keypad
        case 0x62: v = MKC_KP0; break;
        case 0x59: v = MKC_KP1; break;
        case 0x5a: v = MKC_KP2; break;
        case 0x5b: v = MKC_KP3; break;
        case 0x5c: v = MKC_KP4; break;
        case 0x5d: v = MKC_KP5; break;
        case 0x5e: v = MKC_KP6; break;
        case 0x5f: v = MKC_KP7; break;
        case 0x60: v = MKC_KP8; break;
        case 0x61: v = MKC_KP9; break;
        case 0x57: v = MKC_KPAdd; break;
        case 0x56: v = MKC_KPSubtract; break;
        case 0x55: v = MKC_KPMultiply; break;
        case 0x54: v = MKC_KPDevide; break;
        case 0x63: v = MKC_Decimal; break;
    
        default: break;
    }

	return v;
}

GLOBALPROC DoKeyCode(int key_code, blnr down)
{
	ui3r v = HIDKeyCode2MacKeyCode(key_code);
	if (MKC_None != v) {
		Keyboard_UpdateKeyMap2(v, down);
	}
}

LOCALPROC DisableKeyRepeat(void)
{
	/*
		OSGLUxxx common:
		If possible and useful, disable keyboard autorepeat.
	*/
}

LOCALPROC RestoreKeyRepeat(void)
{
	/*
		OSGLUxxx common:
		Undo any effects of DisableKeyRepeat.
	*/
}

LOCALPROC ReconnectKeyCodes3(void)
{
}

LOCALPROC DisconnectKeyCodes3(void)
{
	DisconnectKeyCodes2();
	MyMouseButtonSet(falseblnr);
}

/* --- time, date, location --- */

#define dbglog_TimeStuff (0 && dbglog_HAVE)

LOCALVAR ui5b TrueEmulatedTime = 0;
	/*
		OSGLUxxx common:
		The amount of time the program has
		been running, measured in Macintosh
		"ticks". There are 60.14742 ticks per
		second.

		(time when the emulation is
		stopped for more than a few ticks
		should not be counted.)
	*/

#define HaveWorkingTime 1

#define MyInvTimeDivPow 16
#define MyInvTimeDiv (1 << MyInvTimeDivPow)
#define MyInvTimeDivMask (MyInvTimeDiv - 1)
#define MyInvTimeStep 1089590 /* 1000 / 60.14742 * MyInvTimeDiv */

LOCALVAR uint64_t LastTime;

LOCALVAR uint64_t NextIntTime;
LOCALVAR ui5b NextFracTime;

LOCALPROC IncrNextTime(void)
{
	NextFracTime += MyInvTimeStep;
	NextIntTime += (NextFracTime >> MyInvTimeDivPow);
	NextFracTime &= MyInvTimeDivMask;
}

LOCALPROC InitNextTime(void)
{
	NextIntTime = LastTime;
	NextFracTime = 0;
	IncrNextTime();
}

LOCALVAR ui5b NewMacDateInSeconds;

LOCALFUNC blnr UpdateTrueEmulatedTime(void)
{
	/*
		OSGLUxxx common:
		Update TrueEmulatedTime. Needs to convert between how the host
		operating system measures time and Macintosh ticks.
	*/
	uint64_t LatestTime;
	si5b TimeDiff;

	LatestTime = ESP32API_GetTimeMS( );

	if (LatestTime != LastTime) {

		NewMacDateInSeconds = LatestTime / 1000;
			/* no date and time api in SDL */

		LastTime = LatestTime;
		TimeDiff = (LatestTime - NextIntTime);
			/* this should work even when time wraps */
		if (TimeDiff >= 0) {
			if (TimeDiff > 256) {
				/* emulation interrupted, forget it */
				++TrueEmulatedTime;
				InitNextTime();

#if dbglog_TimeStuff
				dbglog_writelnNum("emulation interrupted",
					TrueEmulatedTime);
#endif
			} else {
				do {
					++TrueEmulatedTime;
					IncrNextTime();
					TimeDiff = (LatestTime - NextIntTime);
				} while (TimeDiff >= 0);
			}
			return trueblnr;
		} else {
			if (TimeDiff < -256) {
#if dbglog_TimeStuff
				dbglog_writeln("clock set back");
#endif
				/* clock goofed if ever get here, reset */
				InitNextTime();
			}
		}
	}

	return falseblnr;
}


LOCALFUNC blnr CheckDateTime(void)
{
	/*
		OSGLUxxx common:
		Update CurMacDateInSeconds, the number
		of seconds since midnight January 1, 1904.

		return true if CurMacDateInSeconds is
		different than it was on the last
		call to CheckDateTime.
	*/

	if (CurMacDateInSeconds != NewMacDateInSeconds) {
		CurMacDateInSeconds = NewMacDateInSeconds;
		return trueblnr;
	} else {
		return falseblnr;
	}
}

LOCALPROC StartUpTimeAdjust(void)
{
	/*
		OSGLUxxx common:
		prepare to call UpdateTrueEmulatedTime.

		will be called again when haven't been
		regularly calling UpdateTrueEmulatedTime,
		(such as the emulation has been stopped).
	*/

	LastTime = ESP32API_GetTimeMS( );
	InitNextTime();
}

LOCALFUNC blnr InitLocationDat(void)
{
#if dbglog_OSGInit
	dbglog_writeln("enter InitLocationDat");
#endif

	LastTime = ESP32API_GetTimeMS( );
	InitNextTime();
	NewMacDateInSeconds = LastTime / 1000;
	CurMacDateInSeconds = NewMacDateInSeconds;

	return trueblnr;
}

/* --- sound --- */

#if MySoundEnabled

#define kLn2SoundBuffers 4 /* kSoundBuffers must be a power of two */
#define kSoundBuffers (1 << kLn2SoundBuffers)
#define kSoundBuffMask (kSoundBuffers - 1)

#define DesiredMinFilledSoundBuffs 3
	/*
		if too big then sound lags behind emulation.
		if too small then sound will have pauses.
	*/

#define kLnOneBuffLen 9
#define kLnAllBuffLen (kLn2SoundBuffers + kLnOneBuffLen)
#define kOneBuffLen (1UL << kLnOneBuffLen)
#define kAllBuffLen (1UL << kLnAllBuffLen)
#define kLnOneBuffSz (kLnOneBuffLen + kLn2SoundSampSz - 3)
#define kLnAllBuffSz (kLnAllBuffLen + kLn2SoundSampSz - 3)
#define kOneBuffSz (1UL << kLnOneBuffSz)
#define kAllBuffSz (1UL << kLnAllBuffSz)
#define kOneBuffMask (kOneBuffLen - 1)
#define kAllBuffMask (kAllBuffLen - 1)
#define dbhBufferSize (kAllBuffSz + kOneBuffSz)

#define dbglog_SoundStuff (0 && dbglog_HAVE)
#define dbglog_SoundBuffStats (0 && dbglog_HAVE)

LOCALVAR tpSoundSamp TheSoundBuffer = nullpr;
volatile static ui4b ThePlayOffset;
volatile static ui4b TheFillOffset;
volatile static ui4b MinFilledSoundBuffs;
#if dbglog_SoundBuffStats
LOCALVAR ui4b MaxFilledSoundBuffs;
#endif
LOCALVAR ui4b TheWriteOffset;

LOCALPROC MySound_Init0(void)
{
	ThePlayOffset = 0;
	TheFillOffset = 0;
	TheWriteOffset = 0;
}

LOCALPROC MySound_Start0(void)
{
	/* Reset variables */
	MinFilledSoundBuffs = kSoundBuffers + 1;
#if dbglog_SoundBuffStats
	MaxFilledSoundBuffs = 0;
#endif
}

GLOBALOSGLUFUNC tpSoundSamp MySound_BeginWrite(ui4r n, ui4r *actL)
{
	ui4b ToFillLen = kAllBuffLen - (TheWriteOffset - ThePlayOffset);
	ui4b WriteBuffContig =
		kOneBuffLen - (TheWriteOffset & kOneBuffMask);

	if (WriteBuffContig < n) {
		n = WriteBuffContig;
	}
	if (ToFillLen < n) {
		/* overwrite previous buffer */
#if dbglog_SoundStuff
		dbglog_writeln("sound buffer over flow");
#endif
		TheWriteOffset -= kOneBuffLen;
	}

	*actL = n;
	return TheSoundBuffer + (TheWriteOffset & kAllBuffMask);
}

#if 4 == kLn2SoundSampSz
LOCALPROC ConvertSoundBlockToNative(tpSoundSamp p)
{
	int i;

	for (i = kOneBuffLen; --i >= 0; ) {
		*p++ -= 0x8000;
	}
}
#else
#define ConvertSoundBlockToNative(p)
#endif

LOCALPROC MySound_WroteABlock(void)
{
#if (4 == kLn2SoundSampSz)
	ui4b PrevWriteOffset = TheWriteOffset - kOneBuffLen;
	tpSoundSamp p = TheSoundBuffer + (PrevWriteOffset & kAllBuffMask);
#endif

#if dbglog_SoundStuff
	dbglog_writeln("enter MySound_WroteABlock");
#endif

	ConvertSoundBlockToNative(p);

	TheFillOffset = TheWriteOffset;

#if dbglog_SoundBuffStats
	{
		ui4b ToPlayLen = TheFillOffset
			- ThePlayOffset;
		ui4b ToPlayBuffs = ToPlayLen >> kLnOneBuffLen;

		if (ToPlayBuffs > MaxFilledSoundBuffs) {
			MaxFilledSoundBuffs = ToPlayBuffs;
		}
	}
#endif
}

LOCALFUNC blnr MySound_EndWrite0(ui4r actL)
{
	blnr v;

	TheWriteOffset += actL;

	if (0 != (TheWriteOffset & kOneBuffMask)) {
		v = falseblnr;
	} else {
		/* just finished a block */

		MySound_WroteABlock();

		v = trueblnr;
	}

	return v;
}

LOCALPROC MySound_SecondNotify0(void)
{
	if (MinFilledSoundBuffs <= kSoundBuffers) {
		if (MinFilledSoundBuffs > DesiredMinFilledSoundBuffs) {
#if dbglog_SoundStuff
			dbglog_writeln("MinFilledSoundBuffs too high");
#endif
			IncrNextTime();
		} else if (MinFilledSoundBuffs < DesiredMinFilledSoundBuffs) {
#if dbglog_SoundStuff
			dbglog_writeln("MinFilledSoundBuffs too low");
#endif
			++TrueEmulatedTime;
		}
#if dbglog_SoundBuffStats
		dbglog_writelnNum("MinFilledSoundBuffs",
			MinFilledSoundBuffs);
		dbglog_writelnNum("MaxFilledSoundBuffs",
			MaxFilledSoundBuffs);
		MaxFilledSoundBuffs = 0;
#endif
		MinFilledSoundBuffs = kSoundBuffers + 1;
	}
}

typedef ui4r trSoundTemp;

#define kCenterTempSound 0x8000

#define AudioStepVal 0x0040

#if 3 == kLn2SoundSampSz
#define ConvertTempSoundSampleFromNative(v) ((v) << 8)
#elif 4 == kLn2SoundSampSz
#define ConvertTempSoundSampleFromNative(v) ((v) + kCenterSound)
#else
#error "unsupported kLn2SoundSampSz"
#endif

#if 3 == kLn2SoundSampSz
#define ConvertTempSoundSampleToNative(v) ((v) >> 8)
#elif 4 == kLn2SoundSampSz
#define ConvertTempSoundSampleToNative(v) ((v) - kCenterSound)
#else
#error "unsupported kLn2SoundSampSz"
#endif

LOCALPROC SoundRampTo(trSoundTemp *last_val, trSoundTemp dst_val,
	tpSoundSamp *stream, int *len)
{
	trSoundTemp diff;
	tpSoundSamp p = *stream;
	int n = *len;
	trSoundTemp v1 = *last_val;

	while ((v1 != dst_val) && (0 != n)) {
		if (v1 > dst_val) {
			diff = v1 - dst_val;
			if (diff > AudioStepVal) {
				v1 -= AudioStepVal;
			} else {
				v1 = dst_val;
			}
		} else {
			diff = dst_val - v1;
			if (diff > AudioStepVal) {
				v1 += AudioStepVal;
			} else {
				v1 = dst_val;
			}
		}

		--n;
		*p++ = ConvertTempSoundSampleToNative(v1);
	}

	*stream = p;
	*len = n;
	*last_val = v1;
}

struct MySoundR {
	tpSoundSamp fTheSoundBuffer;
	volatile ui4b (*fPlayOffset);
	volatile ui4b (*fFillOffset);
	volatile ui4b (*fMinFilledSoundBuffs);

	volatile trSoundTemp lastv;

	blnr wantplaying;
	blnr HaveStartedPlaying;
};
typedef struct MySoundR MySoundR;

#if 0 != SDL_MAJOR_VERSION
static void my_audio_callback(void *udata, Uint8 *stream, int len)
{
	ui4b ToPlayLen;
	ui4b FilledSoundBuffs;
	int i;
	MySoundR *datp = (MySoundR *)udata;
	tpSoundSamp CurSoundBuffer = datp->fTheSoundBuffer;
	ui4b CurPlayOffset = *datp->fPlayOffset;
	trSoundTemp v0 = datp->lastv;
	trSoundTemp v1 = v0;
	tpSoundSamp dst = (tpSoundSamp)stream;

#if kLn2SoundSampSz > 3
	len >>= (kLn2SoundSampSz - 3);
#endif

#if dbglog_SoundStuff
	dbglog_writeln("Enter my_audio_callback");
	dbglog_writelnNum("len", len);
#endif

label_retry:
	ToPlayLen = *datp->fFillOffset - CurPlayOffset;
	FilledSoundBuffs = ToPlayLen >> kLnOneBuffLen;

	if (! datp->wantplaying) {
#if dbglog_SoundStuff
		dbglog_writeln("playing end transistion");
#endif

		SoundRampTo(&v1, kCenterTempSound, &dst, &len);

		ToPlayLen = 0;
	} else if (! datp->HaveStartedPlaying) {
#if dbglog_SoundStuff
		dbglog_writeln("playing start block");
#endif

		if ((ToPlayLen >> kLnOneBuffLen) < 8) {
			ToPlayLen = 0;
		} else {
			tpSoundSamp p = datp->fTheSoundBuffer
				+ (CurPlayOffset & kAllBuffMask);
			trSoundTemp v2 = ConvertTempSoundSampleFromNative(*p);

#if dbglog_SoundStuff
			dbglog_writeln("have enough samples to start");
#endif

			SoundRampTo(&v1, v2, &dst, &len);

			if (v1 == v2) {
#if dbglog_SoundStuff
				dbglog_writeln("finished start transition");
#endif

				datp->HaveStartedPlaying = trueblnr;
			}
		}
	}

	if (0 == len) {
		/* done */

		if (FilledSoundBuffs < *datp->fMinFilledSoundBuffs) {
			*datp->fMinFilledSoundBuffs = FilledSoundBuffs;
		}
	} else if (0 == ToPlayLen) {

#if dbglog_SoundStuff
		dbglog_writeln("under run");
#endif

		for (i = 0; i < len; ++i) {
			*dst++ = ConvertTempSoundSampleToNative(v1);
		}
		*datp->fMinFilledSoundBuffs = 0;
	} else {
		ui4b PlayBuffContig = kAllBuffLen
			- (CurPlayOffset & kAllBuffMask);
		tpSoundSamp p = CurSoundBuffer
			+ (CurPlayOffset & kAllBuffMask);

		if (ToPlayLen > PlayBuffContig) {
			ToPlayLen = PlayBuffContig;
		}
		if (ToPlayLen > len) {
			ToPlayLen = len;
		}

		for (i = 0; i < ToPlayLen; ++i) {
			*dst++ = *p++;
		}
		v1 = ConvertTempSoundSampleFromNative(p[-1]);

		CurPlayOffset += ToPlayLen;
		len -= ToPlayLen;

		*datp->fPlayOffset = CurPlayOffset;

		goto label_retry;
	}

	datp->lastv = v1;
}
#endif /* 0 != SDL_MAJOR_VERSION */

LOCALVAR MySoundR cur_audio;

LOCALVAR blnr HaveSoundOut = falseblnr;

LOCALPROC MySound_Stop(void)
{
#if dbglog_SoundStuff
	dbglog_writeln("enter MySound_Stop");
#endif

	if (cur_audio.wantplaying && HaveSoundOut) {
		ui4r retry_limit = 50; /* half of a second */

		cur_audio.wantplaying = falseblnr;

label_retry:
		if (kCenterTempSound == cur_audio.lastv) {
#if dbglog_SoundStuff
			dbglog_writeln("reached kCenterTempSound");
#endif

			/* done */
		} else if (0 == --retry_limit) {
#if dbglog_SoundStuff
			dbglog_writeln("retry limit reached");
#endif
			/* done */
		} else
		{
			/*
				give time back, particularly important
				if got here on a suspend event.
			*/

#if dbglog_SoundStuff
			dbglog_writeln("busy, so sleep");
#endif

#if 0 != SDL_MAJOR_VERSION
			(void) SDL_Delay(10);
#endif

			goto label_retry;
		}

#if 0 != SDL_MAJOR_VERSION
		SDL_PauseAudio(1);
#endif
	}

#if dbglog_SoundStuff
	dbglog_writeln("leave MySound_Stop");
#endif
}

LOCALPROC MySound_Start(void)
{
	if ((! cur_audio.wantplaying) && HaveSoundOut) {
		MySound_Start0();
		cur_audio.lastv = kCenterTempSound;
		cur_audio.HaveStartedPlaying = falseblnr;
		cur_audio.wantplaying = trueblnr;

#if 0 != SDL_MAJOR_VERSION
		SDL_PauseAudio(0);
#endif
	}
}

LOCALPROC MySound_UnInit(void)
{
	if (HaveSoundOut) {
#if 0 != SDL_MAJOR_VERSION
		SDL_CloseAudio();
#endif
	}
}

#define SOUND_SAMPLERATE 22255 /* = round(7833600 * 2 / 704) */

LOCALFUNC blnr MySound_Init(void)
{
#if dbglog_OSGInit
	dbglog_writeln("enter MySound_Init");
#endif

#if 0 != SDL_MAJOR_VERSION
	SDL_AudioSpec desired;
#endif

	MySound_Init0();

	cur_audio.fTheSoundBuffer = TheSoundBuffer;
	cur_audio.fPlayOffset = &ThePlayOffset;
	cur_audio.fFillOffset = &TheFillOffset;
	cur_audio.fMinFilledSoundBuffs = &MinFilledSoundBuffs;
	cur_audio.wantplaying = falseblnr;

#if 0 != SDL_MAJOR_VERSION
	desired.freq = SOUND_SAMPLERATE;

#if 3 == kLn2SoundSampSz
	desired.format = AUDIO_U8;
#elif 4 == kLn2SoundSampSz
	desired.format = AUDIO_S16SYS;
#else
#error "unsupported audio format"
#endif

	desired.channels = 1;
	desired.samples = 1024;
	desired.callback = my_audio_callback;
	desired.userdata = (void *)&cur_audio;

	/* Open the audio device */
	if (SDL_OpenAudio(&desired, NULL) < 0) {
		fprintf(stderr, "Couldn't open audio: %s\n", SDL_GetError());
	} else {
		HaveSoundOut = trueblnr;

		MySound_Start();
			/*
				This should be taken care of by LeaveSpeedStopped,
				but since takes a while to get going properly,
				start early.
			*/
	}
#endif /* 0 != SDL_MAJOR_VERSION */

	return trueblnr; /* keep going, even if no sound */
}

GLOBALOSGLUPROC MySound_EndWrite(ui4r actL)
{
	if (MySound_EndWrite0(actL)) {
	}
}

LOCALPROC MySound_SecondNotify(void)
{
	/*
		OSGLUxxx common:
		called once a second.
		can be used to check if sound output it
		lagging or gaining, and if so
		adjust emulated time by a tick.
	*/

	if (HaveSoundOut) {
		MySound_SecondNotify0();
	}
}

#endif /* MySoundEnabled */

/* --- basic dialogs --- */

LOCALPROC CheckSavedMacMsg(void)
{
	/*
		OSGLUxxx common:
		This is currently only used in the
		rare case where there is a message
		still pending as the program quits.
	*/

	if (nullpr != SavedBriefMsg) {
		char briefMsg0[ClStrMaxLength + 1];
		char longMsg0[ClStrMaxLength + 1];

		NativeStrFromCStr(briefMsg0, SavedBriefMsg);
		NativeStrFromCStr(longMsg0, SavedLongMsg);

#if 2 == SDL_MAJOR_VERSION
		if (0 != SDL_ShowSimpleMessageBox(
			SDL_MESSAGEBOX_ERROR,
			SavedBriefMsg,
			SavedLongMsg,
			my_main_wind
			))
#endif
		{
			fprintf(stderr, "%s\n", briefMsg0);
			fprintf(stderr, "%s\n", longMsg0);
		}

		SavedBriefMsg = nullpr;
	}
}

/* --- event handling for main window --- */

/*#if UseMotionEvents
LOCALVAR blnr CaughtMouse = falseblnr;
#endif

		case SDL_KEYDOWN:
			DoKeyCode(&event->key.keysym, trueblnr);
			break;
		case SDL_KEYUP:
			DoKeyCode(&event->key.keysym, falseblnr);
			break;
	}
}
#endif */

/* --- main window creation and disposal --- */

LOCALVAR int my_argc;
LOCALVAR char **my_argv;

LOCALFUNC blnr Screen_Init(void)
{
#if dbglog_OSGInit
	dbglog_writeln("enter Screen_Init");
#endif

	InitKeyCodes();

	return trueblnr;
}

#if EnableFSMouseMotion && HaveWorkingWarp
LOCALPROC MyMouseConstrain(void)
{
	si4b shiftdh;
	si4b shiftdv;

	if (SavedMouseH < ViewHStart + (ViewHSize / 4)) {
		shiftdh = ViewHSize / 2;
	} else if (SavedMouseH > ViewHStart + ViewHSize - (ViewHSize / 4)) {
		shiftdh = - ViewHSize / 2;
	} else {
		shiftdh = 0;
	}
	if (SavedMouseV < ViewVStart + (ViewVSize / 4)) {
		shiftdv = ViewVSize / 2;
	} else if (SavedMouseV > ViewVStart + ViewVSize - (ViewVSize / 4)) {
		shiftdv = - ViewVSize / 2;
	} else {
		shiftdv = 0;
	}
	if ((shiftdh != 0) || (shiftdv != 0)) {
		SavedMouseH += shiftdh;
		SavedMouseV += shiftdv;
		if (! MyMoveMouse(SavedMouseH, SavedMouseV)) {
			HaveMouseMotion = falseblnr;
		}
	}
}
#endif

LOCALFUNC blnr CreateMainWindow(void)
{
#if dbglog_OSGInit
	dbglog_writeln("enter CreateMainWindow");
#endif

	return trueblnr;
}

LOCALPROC CloseMainWindow(void)
{
}

LOCALPROC ZapWinStateVars(void)
{
}

#if VarFullScreen
LOCALPROC ToggleWantFullScreen(void)
{
	WantFullScreen = ! WantFullScreen;
}
#endif

/* --- SavedTasks --- */

LOCALPROC LeaveBackground(void)
{
	ReconnectKeyCodes3();
	DisableKeyRepeat();
}

LOCALPROC EnterBackground(void)
{
	RestoreKeyRepeat();
	DisconnectKeyCodes3();

	ForceShowCursor();
}

LOCALPROC LeaveSpeedStopped(void)
{
#if MySoundEnabled
	MySound_Start();
#endif

	StartUpTimeAdjust();
}

LOCALPROC EnterSpeedStopped(void)
{
#if MySoundEnabled
	MySound_Stop();
#endif
}

LOCALPROC CheckForSavedTasks(void)
{
	if (MyEvtQNeedRecover) {
		MyEvtQNeedRecover = falseblnr;

		/* attempt cleanup, MyEvtQNeedRecover may get set again */
		MyEvtQTryRecoverFromFull();
	}

#if EnableFSMouseMotion && HaveWorkingWarp
	if (HaveMouseMotion) {
		MyMouseConstrain();
	}
#endif

	if (RequestMacOff) {
		RequestMacOff = falseblnr;
		if (AnyDiskInserted()) {
			MacMsgOverride(kStrQuitWarningTitle,
				kStrQuitWarningMessage);
		} else {
			ForceMacOff = trueblnr;
		}
	}

	if (ForceMacOff) {
		return;
	}

	if (gTrueBackgroundFlag != gBackgroundFlag) {
		gBackgroundFlag = gTrueBackgroundFlag;
		if (gTrueBackgroundFlag) {
			EnterBackground();
		} else {
			LeaveBackground();
		}
	}

	if (CurSpeedStopped != (SpeedStopped ||
		(gBackgroundFlag && ! RunInBackground
#if EnableAutoSlow && 0
			&& (QuietSubTicks >= 4092)
#endif
		)))
	{
		CurSpeedStopped = ! CurSpeedStopped;
		if (CurSpeedStopped) {
			EnterSpeedStopped();
		} else {
			LeaveSpeedStopped();
		}
	}

	if ((nullpr != SavedBriefMsg) & ! MacMsgDisplayed) {
		MacMsgDisplayOn();
	}

	if (NeedWholeScreenDraw) {
		NeedWholeScreenDraw = falseblnr;
		ScreenChangedAll();
	}

#if NeedRequestIthDisk
	if (0 != RequestIthDisk) {
		Sony_InsertIth(RequestIthDisk);
		RequestIthDisk = 0;
	}
#endif
}

/* --- main program flow --- */

LOCALPROC WaitForTheNextEvent(void)
{
	ESP32API_Yield();
}

LOCALPROC CheckForSystemEvents(void)
{
	/*
		OSGLUxxx common:
		Handle any events that are waiting for us.
		Return immediately when no more events
		are waiting, don't wait for more.
	*/
	ESP32API_CheckForEvents();
	ESP32API_DrawScreen(GetCurDrawBuff());
}

/*
	OSGLUxxx common:
	In general, attempt to emulate one Macintosh tick (1/60.14742
	seconds) for every tick of real time. When done emulating
	one tick, wait for one tick of real time to elapse, by
	calling WaitForNextTick.

	But, Mini vMac can run the emulation at greater than 1x speed, up to
	and including running as fast as possible, by emulating extra cycles
	at the end of the emulated tick. In this case, the extra emulation
	should continue only as long as the current real time tick is not
	over - until ExtraTimeNotOver returns false.
*/

GLOBALOSGLUFUNC blnr ExtraTimeNotOver(void)
{
	UpdateTrueEmulatedTime();
	return TrueEmulatedTime == OnTrueTime;
}

GLOBALOSGLUPROC WaitForNextTick(void)
{
label_retry:
	CheckForSystemEvents();
	CheckForSavedTasks();

	if (ForceMacOff) {
		return;
	}

	if (CurSpeedStopped) {
		DoneWithDrawingForTick();
		WaitForTheNextEvent();
		goto label_retry;
	}

#if ! HaveWorkingTime
	++TrueEmulatedTime;
#endif

	if (ExtraTimeNotOver()) {
		ESP32API_Yield( );
		goto label_retry;
	}

	if (CheckDateTime()) {
#if MySoundEnabled
		MySound_SecondNotify();
#endif
#if EnableDemoMsg
		DemoModeSecondNotify();
#endif
	}

	if ((! gBackgroundFlag)
#if UseMotionEvents
		&& (! CaughtMouse)
#endif
		)
	{
		CheckMouseState();
	}

	OnTrueTime = TrueEmulatedTime;

#if dbglog_TimeStuff
	dbglog_writelnNum("WaitForNextTick, OnTrueTime", OnTrueTime);
#endif
}

/* --- platform independent code can be thought of as going here --- */

#include "PROGMAIN.h"

LOCALPROC ZapOSGLUVars(void)
{
	/*
		OSGLUxxx common:
		Set initial values of variables for
		platform dependent code, where not
		done using c initializers. (such
		as for arrays.)
	*/

	InitDrives();
	ZapWinStateVars();
}

LOCALPROC ReserveAllocAll(void)
{
#if dbglog_HAVE
	dbglog_ReserveAlloc();
#endif
	ReserveAllocOneBlock(&ROM, kROM_Size, 5, falseblnr);

	ReserveAllocOneBlock(&screencomparebuff,
		vMacScreenNumBytes, 5, trueblnr);
#if UseControlKeys
	ReserveAllocOneBlock(&CntrlDisplayBuff,
		vMacScreenNumBytes, 5, falseblnr);
#endif

#if MySoundEnabled
	ReserveAllocOneBlock((ui3p *)&TheSoundBuffer,
		dbhBufferSize, 5, falseblnr);
#endif

	EmulationReserveAlloc();
}

LOCALFUNC blnr AllocMyMemory(void)
{
	uimr n;
	blnr IsOk = falseblnr;

	ReserveAllocOffset = 0;
	ReserveAllocBigBlock = nullpr;
	ReserveAllocAll();
	n = ReserveAllocOffset;
	ReserveAllocBigBlock = (ui3p) ESP32API_calloc(1, n);
	if (NULL == ReserveAllocBigBlock) {
		MacMsg(kStrOutOfMemTitle, kStrOutOfMemMessage, trueblnr);
	} else {
		ReserveAllocOffset = 0;
		ReserveAllocAll();
		if (n != ReserveAllocOffset) {
			/* oops, program error */
		} else {
			IsOk = trueblnr;
		}
	}

	return IsOk;
}

LOCALPROC UnallocMyMemory(void)
{
	if (nullpr != ReserveAllocBigBlock) {
		ESP32API_free((char *)ReserveAllocBigBlock);
	}
}

LOCALFUNC blnr InitOSGLU(void)
{
	/*
		OSGLUxxx common:
		Run all the initializations needed for the program.
	*/

	if (AllocMyMemory())
#if dbglog_HAVE
	if (dbglog_open())
#endif
	if (LoadMacRom())
	if (LoadInitialImages())
	if (InitLocationDat())
#if MySoundEnabled
	if (MySound_Init())
#endif
	if (Screen_Init())
	if (CreateMainWindow())
	if (WaitForRom())
	{
		return trueblnr;
	}
	return falseblnr;
}

LOCALPROC UnInitOSGLU(void)
{
	/*
		OSGLUxxx common:
		Do all clean ups needed before the program quits.
	*/

	if (MacMsgDisplayed) {
		MacMsgDisplayOff();
	}

	RestoreKeyRepeat();
#if MySoundEnabled
	MySound_Stop();
#endif
#if MySoundEnabled
	MySound_UnInit();
#endif
#if IncludePbufs
	UnInitPbufs();
#endif
	UnInitDrives();

	ForceShowCursor();

#if dbglog_HAVE
	dbglog_close();
#endif

	UnallocMyMemory();

	CheckSavedMacMsg();

	CloseMainWindow();
}

int minivmac_main(int argc, char **argv)
{
	my_argc = argc;
	my_argv = argv;

	ZapOSGLUVars();
	if (InitOSGLU()) {
		ProgramMain();
	}
	UnInitOSGLU();

	return 0;
}