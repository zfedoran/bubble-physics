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
    public class Shape
    {
        public Vector2[] points;
        public int count;
        private bool hasBegun;
        private List<Vector2> pointList;
        private bool center;

        public Shape()
        {
            pointList = new List<Vector2>(128);
        }

        public void Begin(bool center)
        {
            if (this.hasBegun)
                throw new Exception("You must call End() before calling Begin()");

            this.hasBegun = true;
            this.pointList.Clear();
            this.center = center;
        }

        public void Add(Vector2 point)
        {
            if (!this.hasBegun)
                throw new Exception("You must call Begin() before adding points");

            pointList.Add(point);
        }

        public void End()
        {
            if (!this.hasBegun)
                throw new Exception("You must call Begin() before calling End()");

            this.hasBegun = false;
            this.points = pointList.ToArray();
            this.count = points.Length;

            if (center)
                CenterAtZero();
        }

        public Shape Clone()
        {
            Shape clone = new Shape();
            clone.count = this.points.Length;
            clone.points = new Vector2[this.points.Length];
            this.points.CopyTo(clone.points,0);
            return clone; 
        }

        public Vector2 GetCenter()
        {
            float x = 0;
            float y = 0;

            for (int i = 0; i < count; i++)
            {
                x += points[i].X;
                y += points[i].Y;
            }

            x = x / count;
            y = y / count;

            return new Vector2(x, y);
        }

        public void CenterAtZero()
        {
            float x = 0;
            float y = 0;
           
            for (int i = 0; i < count; i++)
            {
                x += points[i].X;
                y += points[i].Y;
            }

            x = x / count;
            y = y / count;

            for (int i = 0; i < count; i++)
            {
                points[i].X -= x;
                points[i].Y -= y;
            }
        }

        public static void Transform(ref Vector2[] points, ref Vector2 position, float angle, ref Vector2 scale, out Vector2[] list)
        {
            int count = points.Length;
            Vector2[] array = new Vector2[count];

            for (int i = 0; i < count; i++)
            {
                // rotate the point, and then translate.
                float x = points[i].X * scale.X;
                float y = points[i].Y * scale.Y;
                float c = (float)Math.Cos(angle);
                float s = (float)Math.Sin(angle);
                array[i].X = (c * x) - (s * y) + position.X;
                array[i].Y = (c * y) + (s * x) + position.Y;
            }

            list = array;
        }

    }
}
