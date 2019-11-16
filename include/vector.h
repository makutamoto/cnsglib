#ifndef VECTOR_H
#define VECTOR_H

typedef struct _Vector {
	size_t length;
	struct _VectorItem *firstItem;
	struct _VectorItem *currentItem;
	struct _VectorItem *lastItem;
	struct _VectorItem *cacheItem;
	size_t cacheIndex;
} Vector;

typedef struct _VectorItem {
	struct _VectorItem *previousItem;
	struct _VectorItem *nextItem;
	void *data;
} VectorItem;

typedef struct {
	Vector *vector;
	VectorItem *currentItem;
} VectorIter;

#define iterf(vector, data) for(resetIteration((vector));nextIter((vector), (void**)(data));)

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
