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
    public struct AxisAlignedBoundingBox
    {
        public Vector2 min;
        public Vector2 max;
        public bool valid;

        public AxisAlignedBoundingBox(ref Vector2 min, ref Vector2 max)
        {
            this.min = min;
            this.max = max;
            this.valid = true;
        }

        public AxisAlignedBoundingBox(Vector2 min, Vector2 max)
        {
            this.min = min;
            this.max = max;
            this.valid = true;
        }

        public void Add(float x, float y)
        {
            if (valid)
            {
                if (x < min.X) { min.X = x; }
                else if (x > max.X) { max.X = x; }

                if (y < min.Y) { min.Y = y; }
                else if (y > max.Y) { max.Y = y; }
            }
            else
            {
                min.X = max.X = x;
                min.Y = max.Y = y;
                valid = true;
            }
        }

        public void Clear()
        {
            min.X = max.X = min.Y = max.Y = 0;
            valid = false;
        }

        public bool Contains(float x, float y)
        {
            if (valid)
            {
                if ((x < min.X) || (x > max.X)) return false;
                if ((y < min.Y) || (y > max.Y)) return false;
            }
            else
                return false;
            return true;
        }

        public bool Intersects(ref AxisAlignedBoundingBox aabb)
        {
            // Exit with no intersecton if separated along an axis
            if (this.max.X < aabb.min.X || this.min.X > aabb.max.X) return false;
            if (this.max.Y < aabb.min.Y || this.min.Y > aabb.max.Y) return false;
            // Overlapping on all axis means AABBs are intersecting
            return true;
        }
    }
}
