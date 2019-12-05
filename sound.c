#include<Windows.h>
#include<stdio.h>

#include "./include/borland.h"
#include "./include/sound.h"
#include "./include/vector.h"

#define NAME_SIZE 128

static char name[NAME_SIZE];
static Vector processes;

void initSound(int argc, char *argv[]) {
  if(argc != 1) {
    if(argc == 2) {
      PlaySound(TEXT(argv[1]), NULL, SND_SYNC | SND_FILENAME);
    } else if(argc == 3 && strcmp(argv[1], "LOOP") == 0) {
      PlaySound(TEXT(argv[2]), NULL, SND_LOOP | SND_ASYNC | SND_FILENAME);
    } else {
      ExitProcess(0);
    }
    while(TRUE) Sleep(4294967295);
    ExitProcess(0);
  } else {
    strcat_s(name, sizeof(name), argv[0]);
    strcat_s(name, sizeof(name), " ");
  }
}

Sound PlaySoundNeo(const char path[], int loop) {
	char args[NAME_SIZE] = "";
	STARTUPINFOA startInfo = { 0 };
	PROCESS_INFORMATION processInfo = { 0 };
	startInfo.cb = sizeof(STARTUPINFOA);
  strcat_s(args, sizeof(args), name);
  if(loop) strcat_s(args, sizeof(args), "LOOP ");
	strcat_s(args, sizeof(args), path);
	CreateProcessA(NULL, args, NULL, NULL, FALSE, 0, NULL, NULL, &startInfo, &processInfo);
  CloseHandle(startInfo.hStdInput);
	CloseHandle(startInfo.hStdOutput);
	CloseHandle(startInfo.hStdError);
	CloseHandle(processInfo.hThread);
  pushAlloc(&processes, sizeof(Sound), &processInfo.hProcess);
	return processInfo.hProcess;
}

void StopSound(Sound sound) {
  Sound *soundPointer;
  TerminateProcess(sound, 0);
  CloseHandle(sound);
  iterf(&processes, &soundPointer) {
    if(*soundPointer == sound) {
      removeByData(&processes, soundPointer);
      free(soundPointer);
      break;
    }
  }
}

void deinitSound(void) {
  Sound *sound;
  iterf(&processes, &sound) {
    TerminateProcess(*sound, 0);
    WaitForSingleObject(*sound, INFINITE);
    CloseHandle(*sound);
  }
  freeVector(&processes);
}
