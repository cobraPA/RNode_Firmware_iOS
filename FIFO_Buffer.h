typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
} FIFOBuffer;

inline bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

inline bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;
  
  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline unsigned char fifo_pop(FIFOBuffer *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32
	static inline bool fifo_isempty_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isempty(f);
	  }
	  return result;
	}

	static inline bool fifo_isfull_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isfull(f);
	  }
	  return result;
	}

	static inline void fifo_push_locked(FIFOBuffer *f, unsigned char c) {
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    fifo_push(f, c);
	  }
	}
#endif

/*
static inline unsigned char fifo_pop_locked(FIFOBuffer *f) {
  unsigned char c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo_pop(f);
  }
  return c;
}
*/

inline void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

typedef struct FIFOBuffer16
{
  uint16_t *begin;
  uint16_t *end;
  uint16_t * volatile head;
  uint16_t * volatile tail;
} FIFOBuffer16;

inline bool fifo16_isempty(const FIFOBuffer16 *f) {
  return f->head == f->tail;
}

inline bool fifo16_isfull(const FIFOBuffer16 *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo16_push(FIFOBuffer16 *f, uint16_t c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline uint16_t fifo16_pop(FIFOBuffer16 *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo16_flush(FIFOBuffer16 *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32
	static inline bool fifo16_isempty_locked(const FIFOBuffer16 *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo16_isempty(f);
	  }

	  return result;
	}
#endif

/*
static inline bool fifo16_isfull_locked(const FIFOBuffer16 *f) {
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = fifo16_isfull(f);
  }
  return result;
}


static inline void fifo16_push_locked(FIFOBuffer16 *f, uint16_t c) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    fifo16_push(f, c);
  }
}

static inline size_t fifo16_pop_locked(FIFOBuffer16 *f) {
  size_t c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo16_pop(f);
  }
  return c;
}
*/

inline void fifo16_init(FIFOBuffer16 *f, uint16_t *buffer, uint16_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline uint16_t fifo16_len(FIFOBuffer16 *f) {
  return (f->end - f->begin);
}
