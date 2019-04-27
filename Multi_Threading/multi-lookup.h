#ifndef MULTILOOKUP_H
#define MULTILOOKUP_H


#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <semaphore.h>

#include "PA3/util.h" //make sure file is in correct directory 
#include "PA3/util.c"


struct requestThread
{
    FILE *fData;
    FILE *fLog;
    char *filename;
    char **sharedArray;
    int *index;
    int arraySize, id, filesFinished;
    pthread_mutex_t *mutex;
    sem_t *domain, *space;
};

struct resultThread
{
    char **sharedArray;
    int arraySize,countFiles, id ,*index;
    FILE *fLog;
    char ipString[30];
    FILE **fData;
    pthread_mutex_t *mutex;
    sem_t *domain, *space;
};

void *processRequest(void *arg);
void *processResult(void *arg);
int end_of_files(FILE **files, int countFiles);

#endif
