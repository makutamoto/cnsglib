#include<Windows.h>
#include<stdarg.h>

#include "./include/borland.h"
#include "./include/vector.h"

Vector initVector(void) {
	Vector vector = { 0 };
	return vector;
}

void concatVectorAlloc(Vector *dest, Vector *src, size_t size) {
	VectorItem *item;
	item = src->firstItem;
	while(item) {
		void *data = malloc(size);
		memcpy_s(data, size, item->data, size);
		push(dest, data);
		item = item->nextItem;
	}
}

void concatVector(Vector *dest, Vector *src) {
	VectorItem *item;
	item = src->firstItem;
	while(item) {
		push(dest, item->data);
		item = item->nextItem;
	}
}

void resetIteration(Vector *vector) {
	vector->currentItem = NULL;
}

void* nextData(Vector *vector) {
	if(vector->currentItem == NULL) {
		if(vector->firstItem == NULL) {
			vector->currentItem = NULL;
			return NULL;
		}
		vector->currentItem = vector->firstItem;
	} else {
		if(vector->currentItem->nextItem == NULL) {
			vector->currentItem = NULL;
			return NULL;
		}
		vector->currentItem = vector->currentItem->nextItem;
	}
	return vector->currentItem->data;
}

int nextIter(Vector *vector, void **data) {
	*data = nextData(vector);
	if(*data) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void* previousData(Vector *vector) {
	if(vector->currentItem == NULL) {
		if(vector->lastItem == NULL) {
			vector->currentItem = NULL;
			return NULL;
		}
		vector->currentItem = vector->lastItem;
	} else {
		if(vector->currentItem->previousItem == NULL) {
			vector->currentItem = NULL;
			return NULL;
		}
		vector->currentItem = vector->currentItem->previousItem;
	}
	return vector->currentItem->data;
}

int push(Vector *vector, void *data) {
	VectorItem *newItem = (VectorItem*)malloc(sizeof(VectorItem));
	if(newItem == NULL) return FALSE;
	newItem->previousItem = vector->lastItem;
	newItem->nextItem = NULL;
	newItem->data = data;
	if(vector->firstItem == NULL) vector->firstItem = newItem;
	if(vector->lastItem != NULL) vector->lastItem->nextItem = newItem;
	vector->lastItem = newItem;
	vector->length += 1;
	vector->cacheItem = NULL;
	vector->modifiedCounter += 1;
	return TRUE;
}

int pushAlloc(Vector *vector, size_t size, void *data) {
	void *mem = malloc(size);
	memcpy_s(mem, size, data, size);
	if(push(vector, mem)) return TRUE;
	return FALSE;
}

int pushUntilNull(Vector *vector, ...) {
	va_list ap;
	void *data;
	va_start(ap, vector);
	data = va_arg(ap, void*);
	while(data) {
		if(!push(vector, data)) return FALSE;
		data = va_arg(ap, void*);
	}
	va_end(ap);
	return TRUE;
}

int pushAllocUntilNull(Vector *vector, size_t size, ...) {
	va_list ap;
	void *data;
	va_start(ap, size);
	for(data = va_arg(ap, void*);data != NULL;data = va_arg(ap, void*)) {
		if(!pushAlloc(vector, size, data)) return FALSE;
	}
	va_end(ap);
	return TRUE;
}

void* pop(Vector *vector) {
	VectorItem *previousItem;
	void *data;
	if(vector->lastItem == NULL) return NULL;
	previousItem = vector->lastItem->previousItem;
	data = vector->lastItem->data;
	free(vector->lastItem);
	if(previousItem == NULL) {
		vector->firstItem = NULL;
		vector->currentItem = NULL;
		vector->lastItem = NULL;
	} else {
		if(vector->currentItem == vector->lastItem) vector->currentItem = NULL;
		previousItem->nextItem = NULL;
		vector->lastItem = previousItem;
	}
	vector->length -= 1;
	vector->cacheItem = NULL;
	vector->modifiedCounter += 1;
	return data;
}

VectorItem* ItemAt(Vector *vector, size_t index) {
	size_t i;
	VectorItem *currentItem;
	if(vector->firstItem == NULL) return NULL;
	if(vector->cacheItem && max(vector->cacheIndex, index) - min(vector->cacheIndex, index) < min(index, vector->length - index - 1)) {
		currentItem = vector->cacheItem;
		if(vector->cacheIndex < index) {
			for(i = vector->cacheIndex;i < index;i++) {
				if(currentItem->nextItem == NULL) return NULL;
				currentItem = currentItem->nextItem;
			}
		} else {
			for(i = vector->cacheIndex;i > index;i--) {
				if(currentItem->previousItem == NULL) return NULL;
				currentItem = currentItem->previousItem;
			}
		}
	} else {
		if(index < vector->length / 2) {
			currentItem = vector->firstItem;
			for(i = 0;i < index;i++) {
				if(currentItem->nextItem == NULL) return NULL;
				currentItem = currentItem->nextItem;
			}
		} else {
			size_t count = vector->length - index - 1;
			currentItem = vector->lastItem;
			for(i = 0;i < count;i++) {
				if(currentItem->previousItem == NULL) return NULL;
				currentItem = currentItem->previousItem;
			}
		}
	}
	vector->cacheIndex = index;
	vector->cacheItem = currentItem;
	return currentItem;
}

void* dataAt(Vector *vector, size_t index) {
	VectorItem *item = ItemAt(vector, index);
	if(item == NULL) return NULL;
	return item->data;
}

int insertAt(Vector *vector, size_t index, void *data) {
	if(vector->firstItem  == NULL) {
		VectorItem *newItem;
		if(index != 0) return FALSE;
		newItem = (VectorItem*)malloc(sizeof(VectorItem));
		newItem->previousItem = NULL;
		newItem->nextItem = NULL;
		newItem->data = data;
		vector->firstItem = newItem;
		vector->lastItem = newItem;
	} else {
		if(index == vector->length) {
			push(vector, data);
		} else {
			VectorItem *item = ItemAt(vector, index);
			VectorItem *newItem;
			if(item == NULL) return FALSE;
			newItem = (VectorItem*)malloc(sizeof(VectorItem));
			newItem->previousItem = item->previousItem;
			newItem->nextItem = item;
			newItem->data = data;
			item->previousItem->nextItem = newItem;
			item->previousItem = newItem;
		}
	}
	vector->length += 1;
	vector->cacheItem = NULL;
	vector->modifiedCounter += 1;
	return TRUE;
}

void* removeAt(Vector *vector, size_t index) {
	VectorItem *item = ItemAt(vector, index);
	void *data = item->data;
	if(vector->currentItem == item) vector->currentItem = item->nextItem;
	if(item->previousItem == NULL) {
		vector->firstItem = item->nextItem;
	} else {
		item->previousItem->nextItem = item->nextItem;
	}
	if(item->nextItem == NULL) {
		vector->lastItem = item->previousItem;
	} else {
		item->nextItem->previousItem = item->previousItem;
	}
	free(item);
	vector->length -= 1;
	vector->cacheItem = NULL;
	vector->modifiedCounter += 1;
	return data;
}

void removeByData(Vector *vector, void *data) {
	void *currentData;
	size_t index = 0;
	resetIteration(vector);
	currentData = nextData(vector);
	while(currentData) {
		if(currentData == data) {
			removeAt(vector, index);
		} else {
			index += 1;
		}
		currentData = nextData(vector);
	}
}

void clearVector(Vector *vector) {
	while(vector->length > 0) pop(vector);
}

void freeVector(Vector *vector) {
	while(vector->length > 0) free(pop(vector));
}

VectorIter initVectorIter(Vector *vector) {
	VectorIter iter;
	iter.vector = vector;
	iter.modifiedCounter = vector->modifiedCounter;
	iter.currentItem = NULL;
	return iter;
}

void* nextDataIter(VectorIter *iter) {
	if(iter->modifiedCounter != iter->vector->modifiedCounter) {
		iter->currentItem = NULL;
		iter->modifiedCounter = iter->vector->modifiedCounter;
	} else {
		if(iter->currentItem == NULL) {
			iter->currentItem = iter->vector->firstItem;
		} else {
			iter->currentItem = iter->currentItem->nextItem;
		}
	}
	return iter->currentItem == NULL ? NULL : iter->currentItem->data;
}

void* previousDataIter(VectorIter *iter) {
	if(iter->modifiedCounter != iter->vector->modifiedCounter) {
		iter->currentItem = NULL;
		iter->modifiedCounter = iter->vector->modifiedCounter;
	} else {
		if(iter->currentItem == NULL) {
			iter->currentItem = iter->vector->lastItem;
		} else {
			iter->currentItem = iter->currentItem->previousItem;
		}
	}
	return iter->currentItem == NULL ? NULL : iter->currentItem->data;
}
