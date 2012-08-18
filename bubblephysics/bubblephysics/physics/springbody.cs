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
    public class SpringBody : Body
    {
        public List<Spring> spring_list;
        public List<PointMass> spring_pointmass_list;
        public bool is_constrained = true;
        public float edge_k;
        public float edge_damping;
        public float shape_k;
        public float shape_damping;

        public SpringBody(Shape shape, float mass, float edgeSpringK, float edgeSpringDamp, float shapeSpringK, float shapeSpringDamp)
            : base(shape, mass)
        {
            is_constrained = true;
            spring_list = new List<Spring>();
            spring_pointmass_list = new List<PointMass>();

            shape_k = shapeSpringK;
            shape_damping = shapeSpringDamp;
            edge_k = edgeSpringK;
            edge_damping = edgeSpringDamp;

            // build default springs.
            int i;
            for (i = 0; i < count - 1; i++)
                this.Add(new Spring(pointmass_list[i], pointmass_list[i + 1], edgeSpringK, edgeSpringDamp));
            this.Add(new Spring(pointmass_list[i], pointmass_list[0], edgeSpringK, edgeSpringDamp));
        }

        public void Add(Spring spring)
        {
            if (!pointmass_list.Contains(spring.pointmass_a))
                spring_pointmass_list.Add(spring.pointmass_a);

            if (!pointmass_list.Contains(spring.pointmass_b))
                spring_pointmass_list.Add(spring.pointmass_b);

            spring_list.Add(spring);
        }

        public override void ApplyInternalForces(double elapsed)
        {
            // internal spring forces.
            Vector2 force = new Vector2();
            for (int i = 0; i < spring_list.Count; i++)
            {
                Spring s = spring_list[i];
                Spring.SpringForce(ref s, out force);

                s.pointmass_a.force.X += force.X;
                s.pointmass_a.force.Y += force.Y;
   
                s.pointmass_b.force.X -= force.X;
                s.pointmass_b.force.Y -= force.Y;
            }

            // shape matching forces.
            if (is_constrained)
            {
                for (int i = 0; i < count; i++)
                {
                    if (shape_k > 0)
                    {

                        Spring.SpringForce(ref pointmass_list[i].position, ref pointmass_list[i].velocity, ref curr_shape.points[i],
                                                        ref pointmass_list[i].velocity, 0.0f, shape_k, shape_damping, out force);


                        pointmass_list[i].force.X += force.X;
                        pointmass_list[i].force.Y += force.Y;
                    }
                }
            }

            for (int i = 0; i < spring_pointmass_list.Count; i++)
            {
                this.spring_pointmass_list[i].velocity.X *= damping;
                this.spring_pointmass_list[i].velocity.Y *= damping;
                this.spring_pointmass_list[i].Update(elapsed);
            }
        }
    }
}
