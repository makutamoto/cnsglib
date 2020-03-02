/**
* @file vector.h
* \~english @brief Vector.
* \~japanese @brief â¬ïœí∑îzóÒÅB
*/

#ifndef VECTOR_H
#define VECTOR_H

#include "./borland.h"

typedef struct VectorItem {
	struct VectorItem *previousItem;
	struct VectorItem *nextItem;
	void *data;
} VectorItem;

typedef struct {
	size_t length;
	int modifiedCounter;
	VectorItem *firstItem;
	VectorItem *currentItem;
	VectorItem *lastItem;
	VectorItem *cacheItem;
	size_t cacheIndex;
} Vector;

typedef struct {
	Vector *vector;
	int modifiedCounter;
	VectorItem *currentItem;
} VectorIter;

#define iterf(vector, data) for(resetIteration((vector));nextIter((vector), (void**)(data));)

#define pushAllocStr(vector, str) pushAlloc((vector), sizeof((str)), (str))

Vector initVector(void);
void concatVectorAlloc(Vector *dest, Vector *src, size_t size);
void concatVector(Vector *dest, Vector *src);
void clearVector(Vector *vector);
void freeVector(Vector *vector);

void resetIteration(Vector *vector);
void* nextData(Vector *vector);
int nextIter(Vector *vector, void **data);
void* previousData(Vector *vector);
VectorItem* ItemAt(Vector *vector, size_t index);
void* dataAt(Vector *vector, size_t index);

int push(Vector *vector, void *data);
int pushAlloc(Vector *vector, size_t size, void *data);
int pushUntilNull(Vector *vector, ...);
int pushAllocUntilNull(Vector *vector, size_t size, ...);
void* pop(Vector *vector);
int insertAt(Vector *vector, size_t index, void *data);
void* removeAt(Vector *vector, size_t index);
void removeByData(Vector *vector, void *data);

VectorIter initVectorIter(Vector *vector);
void* nextDataIter(VectorIter *iter);
void* previousDataIter(VectorIter *iter);

#endif
