/*
 * Copyright © 2016-2017 The TokTok team.
 * Copyright © 2013 Tox project.
 * Copyright © 2013 plutooo
 *
 * This file is part of Tox, the free peer to peer instant messenger.
 * This file is donated to the Tox Project.
 *
 * Tox is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Tox is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Tox.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ring_buffer.h"

#include <stdlib.h>

struct RingBuffer {
    uint16_t size; /* Max size */
// Zoff --
    uint8_t data_type; /* 1 -> raw yuv data struct,  0 -> all other data */
// Zoff --
    uint16_t start;
    uint16_t end;
    void   **data;
};

bool rb_full(const RingBuffer *b)
{
    return (b->end + 1) % b->size == b->start;
}

bool rb_empty(const RingBuffer *b)
{
    return b->end == b->start;
}

void *rb_write(RingBuffer *b, void *p, uint8_t data_type_)
{
    void *rc = NULL;

    if ((b->end + 1) % b->size == b->start) { /* full */
        rc = b->data[b->start];
    }

// Zoff --
    b->data_type = data_type_;
// Zoff --

    b->data[b->end] = p;
    b->end = (b->end + 1) % b->size;

    if (b->end == b->start) {
        b->start = (b->start + 1) % b->size;
    }

    return rc;
}

bool rb_read(RingBuffer *b, void **p, uint_8 *data_type_)
{
    if (b->end == b->start) { /* Empty */
        *p = NULL;
        return false;
    }

    *p = b->data[b->start];

// Zoff --
    *data_type_ = b->data_type;
// Zoff --

    b->start = (b->start + 1) % b->size;
    return true;
}

RingBuffer *rb_new(int size)
{
    RingBuffer *buf = (RingBuffer *)calloc(sizeof(RingBuffer), 1);

    if (!buf) {
        return NULL;
    }

    buf->size = size + 1; /* include empty elem */

    if (!(buf->data = (void **)calloc(buf->size, sizeof(void *)))) {
        free(buf);
        return NULL;
    }

    return buf;
}

void rb_kill(RingBuffer *b)
{
    if (b) {
        free(b->data);
        free(b);
    }
}

uint16_t rb_size(const RingBuffer *b)
{
    if (rb_empty(b)) {
        return 0;
    }

    return
        b->end > b->start ?
        b->end - b->start :
        (b->size - b->start) + b->end;
}

uint16_t rb_data(const RingBuffer *b, void **dest)
{
    uint16_t i = 0;

    for (; i < rb_size(b); i++) {
        dest[i] = b->data[(b->start + i) % b->size];
    }

    return i;
}

