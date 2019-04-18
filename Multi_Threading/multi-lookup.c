#include "multi-lookup.h"
#include <sys/time.h>


void *processRequest(void *arg)
{	
	struct requestThread *thread = (struct requestThread *)arg;

	char line[1025];
	int serviced = 0;
	
	while(EOF != fscanf(thread->fData, "%[^\n]\n", line))
	{	
		if(serviced == 0)
		{
			thread->filesFinished++;
			serviced++;
		}

		sem_wait(thread->space);
		pthread_mutex_lock(thread->mutex);

		thread->sharedArray[*thread->index] = strdup(line);

		int space_left, domains_left;
		sem_getvalue(thread->domain, &domains_left);
		sem_getvalue(thread->space, &space_left);
		printf("Requester - id:%d, index:%d, space left:%d, domains left:%d, domain name:%s\n", thread->id, *thread->index, space_left, domains_left+1, line);

		*thread->index = (*thread->index + 1) % thread->arraySize;

		pthread_mutex_unlock(thread->mutex);
		sem_post(thread->domain);

		
	}

	pthread_exit(NULL);
}

void *processResult(void *arg)
{
	struct resultThread *thread = (struct resultThread *)arg;

	char *line;
	int completed;

	while(1)
	{
		sem_getvalue(thread->domain, &completed);
		if(end_of_files(thread->fData, thread->countFiles) && completed == 0)
		{
			break;
		}

		sem_wait(thread->domain);
		pthread_mutex_lock(thread->mutex);

		line = thread->sharedArray[*thread->index];
		thread->sharedArray[*thread->index] = strdup("");

		int space_left, domains_left;
		sem_getvalue(thread->domain, &domains_left);
		sem_getvalue(thread->space, &space_left);
		printf("Resolver - id:%d, index:%d, space left:%d, domains left:%d, domain name:%s\n", thread->id, *thread->index, space_left, domains_left, line);

		int ip = dnslookup(line, thread->ipString, 15);
		if(ip == -1)
		{
			printf("[%s]\n", line);
			fprintf(thread->fLog, "%s,\n", line);
		}
		else
		{
			fprintf(thread->fLog, "%s,%s\n", line, thread->ipString);
		}
		*thread->index = (*thread->index + 1) % thread->arraySize;

		pthread_mutex_unlock(thread->mutex);
		sem_post(thread->space);
	}
	pthread_exit(NULL);
}


int end_of_files(FILE **files, int numFiles)
{
	for(int i = 0; i < numFiles; i++)
	{
		if(!feof(files[i]))
			return 0;
	}

	return 1;
}


int main(int argc, char **argv)
{
    //Start recording time
    struct timeval start, end;
    gettimeofday(&start, NULL);
    
    //Proceess arguments here
    int countRequestThreads = (int) *argv[1] - 48;
    int countResolveThreads = (int) *argv[2] - 48;
    char *requestLogFilename = argv[3];
    char *resolveLogFilename = argv[4];
    int countFiles = argc - 5;
    
    int size = 20;
    char *arr[size];
    int inputArrayIndex = 0;
    int outputArrayIndex = 0;
    
    //Open log files here
    FILE *requestLogFile = fopen(requestLogFilename, "w");
    FILE *resolveLogfile = fopen(resolveLogFilename, "w");
    
    //Handle errors
    if (!requestLogFile ){
        fprintf(stderr, "The program is not able to open the request log file. Exiting ..");
        exit(EXIT_FAILURE);
    }
    if (!resolveLogfile){
        fprintf(stderr, "The program is not able to open the resolve log file. Exiting ..");
        exit(EXIT_FAILURE);
    }
    
    int failedToOpenCount = 0;
    for (int i =5; i< countFiles+5; i++)
        if (fopen(argv[i], "r") == NULL)
            failedToOpenCount++;
    
    //Create an array of files that has been opened already
    countFiles -= failedToOpenCount;
    FILE **dataFiles = malloc(sizeof(FILE*) * countFiles);
    for (int i =0; i<countFiles; i++)
        if (fopen(argv[i+5],"r") != NULL)
            dataFiles[i] = fopen(argv[i+5], "r");
    
        
    //Start working on currency by initializing mutexes and semaphores
    pthread_mutex_t array_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t array_mutex2 = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t request_mutex = PTHREAD_MUTEX_INITIALIZER;
    sem_t domain;
    sem_t space;
    sem_init(&domain, 0,0);
    sem_init(&space, 0, 20);
    
    //We initialize requester threads over here
    pthread_t *requestId = malloc(countRequestThreads * (sizeof(pthread_t)));
    struct requestThread argsRequest[countRequestThreads];
    
    //We initialize resolver threads over here
    pthread_t *resolveId = malloc(countResolveThreads * (sizeof(pthread_t)));
    struct resultThread argsResolve[countResolveThreads];
    
    //Initialize filesFinished in request struct
    for (int i = 0; i < countRequestThreads; i++)
        argsRequest[i].filesFinished = 0;
    if (countRequestThreads < countFiles)
    {
        int j = 0;
        int tempThreadNum;
        for (int i =0; i < countFiles; i++)
        {
            argsRequest[j].fData = dataFiles[i];
            argsRequest[j].filename = argv[i+5];
            argsRequest[j].sharedArray = arr;
            argsRequest[j].index = &inputArrayIndex;
            argsRequest[j].arraySize = size;
            argsRequest[j].id = i;
            argsRequest[j].fLog = requestLogFile;
            argsRequest[j].mutex = &array_mutex;
            argsRequest[j].domain = &domain;
            argsRequest[j].space = &space;
            pthread_create(&requestId[j], NULL,processRequest, &argsRequest[j]);
            
            tempThreadNum++;
            
            if ((j == countRequestThreads - 1) || (i == countFiles - 1)){
                j = -1;
                for (int i =0; i<countResolveThreads; i++){
                    argsResolve[i].sharedArray = arr;
					argsResolve[i].index = &outputArrayIndex;
					argsResolve[i].arraySize = size;
					argsResolve[i].fLog = resolveLogfile;
					argsResolve[i].fData = dataFiles;
					argsResolve[i].countFiles = countRequestThreads;
					argsResolve[i].mutex = &array_mutex2;
					argsResolve[i].domain = &domain;
					argsResolve[i].space = &space;
					argsResolve[i].id = i;
                    pthread_create(&resolveId[i], NULL, processResult, &argsResolve[i]);
                }
                
                for (int i =0; i < tempThreadNum; i++)
                    pthread_join(requestId[i], NULL);
                for (int i =0; i < countResolveThreads; i++)
                    pthread_join(resolveId[i], NULL);
                
                tempThreadNum = 0;
            }
            j++;
        }
    }
    
    else 
    {
        int index = 0;
        for (int i =0; i< countRequestThreads; i++)
        {
            if (index > countFiles - 1)
                index = 0;
            argsRequest[i].fData = dataFiles[index];
			argsRequest[i].filename = argv[index + 5];
			argsRequest[i].sharedArray = arr;
			argsRequest[i].index = &inputArrayIndex;
			argsRequest[i].arraySize = size;
			argsRequest[i].id = i;
			argsRequest[i].fLog = requestLogFile;
			argsRequest[i].mutex = &array_mutex;
			argsRequest[i].domain = &domain;
			argsRequest[i].space = &space;
			pthread_create(&requestId[i], NULL, processRequest, &argsRequest[i]);

            index ++;
        }
        
        for (int i =0; i < countResolveThreads; i++)
        {
            argsResolve[i].sharedArray = arr;
			argsResolve[i].index = &outputArrayIndex;
			argsResolve[i].arraySize = size;
			argsResolve[i].fLog = resolveLogfile;
			argsResolve[i].fData = dataFiles;
			argsResolve[i].countFiles = countFiles;
			argsResolve[i].mutex = &array_mutex2;
			argsResolve[i].domain = &domain;
			argsResolve[i].space = &space;
			argsResolve[i].id = i;
            pthread_create(&resolveId[i], NULL, processResult, &argsResolve[i]);
        }
        
        for (int i =0; i<countRequestThreads; i++)
            pthread_join(requestId[i],NULL);
        for (int i = 0; i < countResolveThreads; i++)
            pthread_join(resolveId[i], NULL);
        
    }
    
    
    for (int i = 0; i< countRequestThreads; i++)
    {
        pthread_mutex_lock(&request_mutex);
        fprintf(requestLogFile, "Thread %d finished %d files\n", i, argsRequest[i].filesFinished);
        pthread_mutex_unlock(&request_mutex);
    }
    
    //Finish time here
    gettimeofday(&end, NULL);
    double runtime = (end.tv_sec - start.tv_sec) + ((end.tv_usec - start.tv_usec)/1000000.0);
	printf("Number for requester thread = %s\n", argv[1]);
	printf("Number for resolver thread = %s\n", argv[2]);
	for(int i = 0; i < countRequestThreads; i++)
        printf("Thread %d serviced %d files\n", i, argsRequest[i].filesFinished);
	
	printf("Total run time: %f\n\n", runtime);

	fclose(requestLogFile);
	fclose(resolveLogfile);
	for(int i = 0; i < countFiles; i++)
		fclose(dataFiles[i]);
	
	free(requestId);
    free(resolveId);

}
