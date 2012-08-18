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
using Microsoft.Xna.Framework;

namespace bubblephysics.physics
{
    public class Spring
    {
        public float damping;
        public float k;
        public float d;
        public PointMass pointmass_a;
        public PointMass pointmass_b;

        public Spring(PointMass pointmass_a, PointMass pointmass_b, float k, float damping)
            : this(pointmass_a, pointmass_b, k, damping, 0)
        { Reset(); }

        public Spring(PointMass pointmass_a, PointMass pointmass_b, float k, float damping, float length)
        {
            this.pointmass_a = pointmass_a;
            this.pointmass_b = pointmass_b;
            this.d = length;
            this.k = k;
            this.damping = damping;
        }

        public override string ToString()
        {
            return string.Format("{{a:[{0}] b:[{1}] length:{2}}}", pointmass_a, pointmass_b, d);
        }

        public void Reset()
        { d = (pointmass_a.position - pointmass_b.position).Length(); }


        public static void SpringForce(ref Spring spring, out Vector2 forceOut)
        {
            SpringForce(ref spring.pointmass_a.position, ref spring.pointmass_a.velocity, ref spring.pointmass_b.position, ref spring.pointmass_b.velocity, spring.d, spring.k, spring.damping, out forceOut);
        }

        public static void SpringForce(ref Vector2 posA, ref Vector2 velA, ref Vector2 posB, ref Vector2 velB, float springD, float springK, float damping, out Vector2 forceOut)
        {
#if XBOX360
            forceOut = new Vector2();
#endif

            float BtoAX = (posA.X - posB.X);
            float BtoAY = (posA.Y - posB.Y);

            float dist = (float)Math.Sqrt((BtoAX * BtoAX) + (BtoAY * BtoAY));
            if (dist > 0.0001f)
            {
                BtoAX /= dist;
                BtoAY /= dist;
            }
            else
            {
                forceOut.X = 0;
                forceOut.Y = 0;
                return;
            }

            dist = springD - dist;

            float relVelX = velA.X - velB.X;
            float relVelY = velA.Y - velB.Y;

            float totalRelVel = (relVelX * BtoAX) + (relVelY * BtoAY);

            forceOut.X = BtoAX * ((dist * springK) - (totalRelVel * damping));
            forceOut.Y = BtoAY * ((dist * springK) - (totalRelVel * damping));
        }
    }
}
