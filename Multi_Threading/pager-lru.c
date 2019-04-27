/*
 * File: pager-lru.c
 * Author:       Andy Sayler
 *               http://www.andysayler.com
 * Adopted From: Dr. Alva Couch
 *               http://www.cs.tufts.edu/~couch/
 *
 * Project: CSCI 3753 Programming Assignment 4
 * Create Date: Unknown
 * Modify Date: 2012/04/03
 * Description:
 * 	This file contains an lru pageit
 *      implmentation.
 */

 // Adapted from: https://github.com/Prasad1337/paging-lab/blob/master/pager-lru.c

#include <stdio.h> 
#include <stdlib.h>

#include "simulator.h"


 /* Static vars */
static int initialized = 0;
static int tick = 0;    //artificial time
static int timestamps[MAXPROCESSES][MAXPROCPAGES];
static int freq[MAXPROCPAGES][MAXPROCPAGES];
static int prev[MAXPROCESSES];

/* Local vars */
int proctmp;
int pagetmp;


static void lru_page(Pentry q[MAXPROCESSES], int proc, int tick, int *pgAddress)
{

    
    
	int t = tick + 1;
	*pgAddress = -1;

	for (int pg = 0; pg < MAXPROCPAGES; pg++)
	{
		// check if page is swapped out
		if (!q[proc].pages[pg])
			continue;

		if (timestamps[proc][pg] < t)
		{
			t = timestamps[proc][pg];
			*pgAddress = pg;

			if (t <= 1) break;
		}
	}
	
	
        

    
    
}

void pageit(Pentry q[MAXPROCESSES])
{
	if (!initialized) {
		for (proctmp = 0; proctmp < MAXPROCESSES; proctmp++) {
			for (pagetmp = 0; pagetmp < MAXPROCPAGES; pagetmp++) {
				timestamps[proctmp][pagetmp] = 0;
                freq[proctmp][pagetmp]=0;
			}
			prev[proctmp] = -1;
		}
		initialized = 1;
	}
	

	/* TODO: Implement LRU Paging */
	//fprintf(stderr, "pager-lru not yet implemented. Exiting...\n");
	//exit(EXIT_FAILURE);

	int currentPG;
	int pgAddress;

	for (int pid = 0; pid < MAXPROCESSES; pid++)
	{
		// determine current page size & record timestamp
		currentPG = q[pid].pc / PAGESIZE;
		timestamps[pid][currentPG] = tick;

		if (pagein(pid, currentPG)){
            if(prev[pid] != -1 && prev[pid] != currentPG)
                    {
                        freq[prev[pid]][currentPG]++;
                    }
            prev[pid] = currentPG;
            continue;
        }
			

		lru_page(q, pid, tick, &pgAddress);
		pageout(pid, pgAddress);
	}

	if(tick % 10000 == 0 ) {
        printf("{\n");
        for(int i=0; i < MAXPROCPAGES; i++)
        {
            printf("\t{");
            for(int j=0; j < MAXPROCPAGES; j++)
            {
                printf("%3d,",freq[i][j]);
            }
            printf("},\n");

        }
        printf("}\n");
    }
	/* advance time for next pageit iteration */
	tick++;
}
