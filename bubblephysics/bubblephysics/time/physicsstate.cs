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
using Microsoft.Xna.Framework;

namespace bubblephysics.time
{
    public class PhysicsState
    {
        public Body body;
        public Vector2[] pointmass_list_positions;
        public Vector2[] pointmass_list_velocities;
        public Vector2[] pointmass_list_forces;
        public Vector2[] curr_shape_positions;
        public float time;

        public PhysicsState(Body body)
        {
            this.body = body;
            this.time = 0;
            this.pointmass_list_positions = new Vector2[body.count];
            this.pointmass_list_velocities = new Vector2[body.count];
            this.pointmass_list_forces = new Vector2[body.count];
            this.curr_shape_positions = new Vector2[body.count];
        }

        public void Save(float time)
        {
            this.time = time;

            for (int i = 0; i < body.count; i++)
            {
                this.curr_shape_positions[i].X = body.curr_shape.points[i].X;
                this.curr_shape_positions[i].Y = body.curr_shape.points[i].Y;

                PointMass pointmass = body.pointmass_list[i];

                this.pointmass_list_positions[i].X = pointmass.position.X;
                this.pointmass_list_positions[i].Y = pointmass.position.Y;

                this.pointmass_list_velocities[i].X = pointmass.velocity.X;
                this.pointmass_list_velocities[i].Y = pointmass.velocity.Y;

                this.pointmass_list_forces[i].X = pointmass.force.X;
                this.pointmass_list_forces[i].Y = pointmass.force.Y;

            }
        }

        public override string ToString()
        {
            return string.Format("{0}", time);
        }

        public static void Interpolate(PhysicsState a, PhysicsState b, float time)
        {
            float amount = (time - a.time) / (b.time - a.time);
            Body body = a.body;

            for (int i = 0; i < body.count; i++)
            {
                body.curr_shape.points[i].X = MathHelper.Lerp(a.curr_shape_positions[i].X, b.curr_shape_positions[i].X, amount);
                body.curr_shape.points[i].Y = MathHelper.Lerp(a.curr_shape_positions[i].Y, b.curr_shape_positions[i].Y, amount);

                PointMass pointmass = body.pointmass_list[i];

                pointmass.position.X = MathHelper.Lerp(a.pointmass_list_positions[i].X, b.pointmass_list_positions[i].X, amount);
                pointmass.position.Y = MathHelper.Lerp(a.pointmass_list_positions[i].Y, b.pointmass_list_positions[i].Y, amount);

                pointmass.velocity.X = MathHelper.Lerp(a.pointmass_list_velocities[i].X, b.pointmass_list_velocities[i].X, amount);
                pointmass.velocity.Y = MathHelper.Lerp(a.pointmass_list_velocities[i].Y, b.pointmass_list_velocities[i].Y, amount);

                pointmass.force.X = MathHelper.Lerp(a.pointmass_list_forces[i].X, b.pointmass_list_forces[i].X, amount);
                pointmass.force.Y = MathHelper.Lerp(a.pointmass_list_forces[i].Y, b.pointmass_list_forces[i].Y, amount);

            }

            body.UpdateBodyPositionVelocityForce(0);
        }
    }
}
