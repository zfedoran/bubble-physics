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
    public class PressureBody : SpringBody
    {
        internal float volume;
        internal float pressure;
        internal Vector2[] normal_list;
        internal float[] edgelength_list;


        public PressureBody(Shape s, float mass, float gasPressure, float edgeSpringK, float edgeSpringDamp, float shapeSpringK, float shapeSpringDamp)
            : base(s, mass, edgeSpringK, edgeSpringDamp, shapeSpringK, shapeSpringDamp)
        {
            pressure = gasPressure;
            normal_list = new Vector2[count];
            edgelength_list = new float[count];
        }

        public override void ApplyInternalForces(double elapsed)
        {
            base.ApplyInternalForces(elapsed);

            // internal forces based on pressure equations.  we need 2 loops to do this.  one to find the overall volume of the
            // body, and 1 to apply forces.  we will need the normals for the edges in both loops, so we will cache them and remember them.
            volume = 0f;

            for (int i = 0; i < count; i++)
            {
                int prev = (i > 0) ? i - 1 : count - 1;
                int next = (i < count - 1) ? i + 1 : 0;

                // currently we are talking about the edge from i --> j.
                // first calculate the volume of the body, and cache normals as we go.
                Vector2 edge1N = new Vector2();
                edge1N.X = pointmass_list[i].position.X - pointmass_list[prev].position.X;
                edge1N.Y = pointmass_list[i].position.Y - pointmass_list[prev].position.Y;
                VectorHelper.Perpendicular(ref edge1N);

                Vector2 edge2N = new Vector2();
                edge2N.X = pointmass_list[next].position.X - pointmass_list[i].position.X;
                edge2N.Y = pointmass_list[next].position.Y - pointmass_list[i].position.Y;
                VectorHelper.Perpendicular(ref edge2N);

                Vector2 norm = new Vector2();
                norm.X = edge1N.X + edge2N.X;
                norm.Y = edge1N.Y + edge2N.Y;

                float nL = (float)Math.Sqrt((norm.X * norm.X) + (norm.Y * norm.Y));
                if (nL > 0.001f)
                {
                    norm.X /= nL;
                    norm.Y /= nL;
                }

                float edgeL = (float)Math.Sqrt((edge2N.X * edge2N.X) + (edge2N.Y * edge2N.Y));

                // cache normal and edge length
                normal_list[i] = norm;
                edgelength_list[i] = edgeL;

                float xdist = Math.Abs(pointmass_list[i].position.X - pointmass_list[next].position.X);

                float volumeProduct = xdist * Math.Abs(norm.X) * edgeL;

                // add to volume
                volume += 0.5f * volumeProduct;
            }

            // now loop through, adding forces!
            float invVolume = 1f / volume;

            for (int i = 0; i < count; i++)
            {
                int j = (i < count - 1) ? i + 1 : 0;

                float pressureV = (invVolume * edgelength_list[i] * pressure);
                pointmass_list[i].force.X += normal_list[i].X * pressureV;
                pointmass_list[i].force.Y += normal_list[i].Y * pressureV;
                                  
                pointmass_list[j].force.X += normal_list[j].X * pressureV;
                pointmass_list[j].force.Y += normal_list[j].Y * pressureV;
            }
        }
    }
}
