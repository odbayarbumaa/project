/*
 * File: pager-predict.c
 * Author:       Andy Sayler
 *               http://www.andysayler.com
 * Adopted From: Dr. Alva Couch
 *               http://www.cs.tufts.edu/~couch/
 *
 * Project: CSCI 3753 Programming Assignment 4
 * Create Date: Unknown
 * Modify Date: 2012/04/03
 * Description:
 * 	This file contains a predictive pageit
 *      implmentation.
 */


#include <stdio.h> 
#include <stdlib.h>

#include "simulator.h"

void pageit(Pentry q[MAXPROCESSES]) { 

static double pagePredict[MAXPROCESSES][MAXPROCPAGES] = {

	{  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{.75,  0,  0,  0,  0,  0,  0,  0,  0,.25,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,1,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},

    };

    for(int proc=0;proc < MAXPROCESSES;proc++){
        
        int pagesToSwap[MAXPROCPAGES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Pages that needs swapping within this process. If an element is 1 that means the page is at the index needs swapping in otherwise we should swapping out
            
        int curPage = q[proc].pc / PAGESIZE;

        pagesToSwap[curPage] = 1; // Set current page to 1. Meaning that it needs to be swapped in the memory

        // Even if there is a low probability of the neighbor being next, add to the array to reduce thrashing
        for(int frame = 0; frame < MAXPROCPAGES; frame++){
            
            //Will swap pages even the ones with low probability so we can avoid spending too much time on page faults
            if(pagePredict[curPage][frame] > 0)
                pagesToSwap[frame] = 1;
            
            if(pagePredict[curPage][frame] > .74) 
                // if a page has a high probability that means its neighbors are probably gonna have a probability higher than therefore need to swap in
                for(int neighbor = 0; neighbor < MAXPROCPAGES; neighbor++)
                    if(pagePredict[frame][neighbor] > 0)
                        pagesToSwap[neighbor] = 1;       
        }
        // This is more simplified than LRU
        // All pages that are not needed within the next three jumps are swapped out to free space
        for(int frame = 0; frame < MAXPROCPAGES; frame++)
            //If frame is 1 within the array then swap in otherwise swap out
            if(pagesToSwap[frame] == 0)
                pageout(proc,frame);
            else
                pagein(proc, frame);
     
} 
}
