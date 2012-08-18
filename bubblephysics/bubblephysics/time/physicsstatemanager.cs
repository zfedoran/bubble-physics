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
    public class PhysicsStateManager
    {
        private Physics physics;
        private Dictionary<Body, PhysicsHistory> bodies;
        private float frequency;
        private float curr_time;

        public PhysicsStateManager(Physics physics)
        {
            this.curr_time = 0f;
            this.frequency = 1/15f;
            this.physics = physics;
            this.bodies = new Dictionary<Body, PhysicsHistory>();
        }

        public void SetRecordingFrequency(float frequency)
        {
            this.frequency = frequency;
        }

        public int GetCurrentIndex(Body body)
        {
            if(bodies.ContainsKey(body))
                return bodies[body].GetCurrentIndex();

            return -1;
        }

        public void Save(float time)
        {
            if (time - curr_time < frequency)
                return;
            curr_time = time;

            for (int i = 0; i < physics.body_list.Count; i++)
            {
                Body body = physics.body_list[i];
                PhysicsHistory history;

                if (!bodies.ContainsKey(body))
                {
                    history = new PhysicsHistory(body);
                    bodies.Add(body, history);
                }
                else
                {
                    history = bodies[body]; 
                }

                history.Save(time);
            }
        }

        public void RewindTo(float time)
        {
            curr_time = time;

            for (int i = 0; i < physics.body_list.Count; i++)
            {
                Body body = physics.body_list[i];
                PhysicsHistory history;

                if (bodies.ContainsKey(body))
                {
                    history = bodies[body];
                    history.RewindTo(time);
                }
            }
        }
    }
}
