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
    public class PointMass
    {
        public float mass;
        public Vector2 position;
        public Vector2 velocity;
        public Vector2 force;

        public PointMass()
        { }

        public PointMass(Vector2 pos, float mass)
        {
            this.mass = mass;
            this.position = pos;
            this.velocity = force = Vector2.Zero;
        }

        public override string ToString()
        {
            return string.Format("{{position:[{0}] velocity:[{1}] force:[{2}]}}", position, velocity, force);
        }

        public void Update(double elapsed)
        {
            float k = (float) elapsed / mass;

            velocity.X += (force.X * k);
            velocity.Y += (force.Y * k);

            position.X += (velocity.X * k);
            position.Y += (velocity.Y * k);

            force.X = 0f;
            force.Y = 0f;
        }
    }
}
