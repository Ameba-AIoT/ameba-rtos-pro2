/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * All rights reserved.
 */

#include <stdlib.h>
#include <stdio.h>

#include "wsfs_internal.h"

#undef WSFS_DEBUG_MEM

#define MAX_ALLOC_SIZE (5*1024*1024) // 5 MB

void *wsfs_realloc(void *ptr, size_t new_size, const char *func)
{
	if (new_size == 0 || new_size > MAX_ALLOC_SIZE) {
		printf("[WSFS_MEM] invalid new_size: %zu, caller: %s\n", new_size, func);
		return NULL;
	}

	void *resized = realloc(ptr, new_size);
#ifdef WSFS_DEBUG_MEM
	if (resized) {
		printf("[WSFS_MEM] realloc: %p -> %p, size: %zu, caller: %s\n", ptr, resized, new_size, func);
	} else {
		printf("[WSFS_MEM] realloc failed: %p, size: %zu, caller: %s\n", ptr, new_size, func);
	}
#endif
	return resized;
}

void *wsfs_malloc(size_t size, const char *func)
{
	if (size == 0 || size > MAX_ALLOC_SIZE) {
		printf("[WSFS_MEM] invalid size: %zu, caller: %s\n", size, func);
		return NULL;
	}

	void *ptr = malloc(size);
#ifdef WSFS_DEBUG_MEM
	if (ptr) {
		printf("[WSFS_MEM] malloc: %p, size: %zu, caller: %s\n", ptr, size, func);
	} else {
		printf("[WSFS_MEM] malloc failed: size: %zu, caller: %s\n", size, func);
	}
#endif
	return ptr;
}

void wsfs_free(void *ptr, const char *func)
{
#ifdef WSFS_DEBUG_MEM
	if (ptr) {
		printf("[WSFS_MEM] free: %p, caller: %s\n", ptr, func);
	}
#endif
	free(ptr);
}
