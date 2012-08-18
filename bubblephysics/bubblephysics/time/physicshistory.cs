/*
 * Copyright (C) 2009-2012 - Zelimir Fedoran
 *
 * This file is part of Bubble Physics.
 *
 * Bubble Physics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Bubble Physics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Bubble Physics.  If not, see <http://www.gnu.org/licenses/>.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using bubblephysics.physics;

namespace bubblephysics.time
{
    public class PhysicsHistory
    {
        private PhysicsState[] states;
        private int capacity;
        private int curr_index;
        private Body body;

        public PhysicsHistory(Body body){
            this.capacity = 512;
            this.curr_index = 0;
            this.body = body;
            this.states = new PhysicsState[capacity];

            for (int i = 0; i < states.Length; i++)
                states[i] = new PhysicsState(body);
        }

        public int GetCurrentIndex()
        {
            return curr_index;
        }

        public void Save(float time)
        {
            states[curr_index].Save(time);
            curr_index++;
            curr_index %= capacity;
        }

        public void RewindTo(float time)
        {
            PhysicsState next, prev, curr;
            next = curr = prev = null;

            for (int i = 0; i < capacity; i++)
            {
                curr = states[i];
                if (time > curr.time && (prev == null || prev.time < curr.time))
                {
                    prev = curr;
                    curr_index = (i + 1) % capacity;
                }

                if (time <= curr.time && (next == null || next.time >= curr.time))
                    next = curr;
            }

            if(next == null || next.time < prev.time)
                PhysicsState.Interpolate(prev, prev, time);
            else
                PhysicsState.Interpolate(prev, next, time);
        }
    }
}
