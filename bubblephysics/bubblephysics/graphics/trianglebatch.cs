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
using Microsoft.Xna.Framework.Graphics;

namespace bubblephysics.graphics
{
    public class TriangleBatch
    {
        private GraphicsDevice device;
        private TriangleVertex[] vertices;
        private int curr_vertex;
        private bool hasBegun;

        public TriangleBatch(GraphicsDevice device)
        {
            this.device = device;
            this.vertices = new TriangleVertex[512];
            this.curr_vertex = 0;
            this.hasBegun = false;
        }

        public void Add(TriangleVertex vertex)
        {
            if (!this.hasBegun)
                throw new Exception("You must call Begin() first");

            if (this.curr_vertex >= this.vertices.Length)
            {
                TriangleVertex[] tmp = new TriangleVertex[this.vertices.Length + 512];
                this.vertices.CopyTo(tmp, 0);
                this.vertices = tmp;
            }

            this.vertices[this.curr_vertex++] = vertex;
        }

        public void Begin()
        {
            if (this.hasBegun)
                throw new Exception("You must call End() first");

            this.hasBegun = true;
            this.curr_vertex = 0;
        }

        public void End()
        {
            if (!this.hasBegun)
                throw new Exception("You must call Begin() first");
            if (!(this.curr_vertex % 3 == 0))
                throw new Exception("You have not added enough vertices");

            this.hasBegun = false;

            device.DrawUserPrimitives<TriangleVertex>(PrimitiveType.TriangleList, this.vertices, 0, (this.curr_vertex + 1) / 3);
        }
    }
}
