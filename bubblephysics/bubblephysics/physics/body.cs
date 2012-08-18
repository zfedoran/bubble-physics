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
    public class Body
    {
        public Shape base_shape;
        public Shape curr_shape;
        public PointMass[] pointmass_list;
        public AxisAlignedBoundingBox aabb;
        public Vector2 scale = Vector2.One;
        public Vector2 position;
        public Vector2 velocity;
        public Vector2 force;
        public int count;
        public float curr_angle;
        public float prev_angle;
        public float omega;
        public float damping = 0.999f;
        public bool is_static = false;
        public bool is_dirty = true;
        public bool is_merging = false;
        public Bitmask bitmaskx;
        public Bitmask bitmasky;

        public Body(Shape shape, float mass)
        {
            this.base_shape = shape;
            this.curr_shape = shape.Clone();
            this.count = shape.count;

            this.pointmass_list = new PointMass[shape.count];
            for (int i = 0; i < shape.count; i++)
                pointmass_list[i] = new PointMass(shape.points[i], mass);

            this.bitmaskx = new Bitmask();
            this.bitmasky = new Bitmask();
        }

        private void UpdatePointMasses(double elapsed)
        {
            for (int i = 0; i < count; i++)
            {
                pointmass_list[i].velocity.X *= damping;
                pointmass_list[i].velocity.Y *= damping;
                pointmass_list[i].Update(elapsed);
            }
        }

        private void UpdataAABB(double elapsed)
        {
            aabb.Clear();

            float x, y;
            for (int i = 0; i < count; i++)
            {
                x = pointmass_list[i].position.X;
                y = pointmass_list[i].position.Y;

                aabb.Add(x, y);

                x += (float)(pointmass_list[i].velocity.X * elapsed);
                y += (float)(pointmass_list[i].velocity.Y * elapsed);

                aabb.Add(x, y);
            }
        }

        private void SetBodyPositionVelocityForce(Vector2 position, Vector2 velocity, Vector2 force)
        {
            float inverse_count = 1.0f / count;

            Vector2 curr_position, curr_velocity, curr_force;
            GetBodyPositionVelocityForce(out curr_position, out curr_velocity, out curr_force);

            for (int i = 0; i < count; i++)
            {
                pointmass_list[i].position -= curr_position; //convert to local coords
                pointmass_list[i].position += position;

                pointmass_list[i].velocity -= curr_velocity;
                pointmass_list[i].velocity += velocity;

                pointmass_list[i].force -= curr_force;
                pointmass_list[i].force += force;
            }
        }

        private void GetBodyPositionVelocityForce(out Vector2 position, out Vector2 velocity, out Vector2 force)
        {
#if XBOX360
            position = new Vector2();
            velocity = new Vector2();
            force = new Vector2();
#endif
            float inverse_count = 1.0f / count;

            position.X = 0;
            position.Y = 0;

            velocity.X = 0;
            velocity.Y = 0;

            force.X = 0;
            force.Y = 0;

            for (int i = 0; i < count; i++)
            {
                position.X += pointmass_list[i].position.X * inverse_count;
                position.Y += pointmass_list[i].position.Y * inverse_count;

                velocity.X += pointmass_list[i].velocity.X * inverse_count;
                velocity.Y += pointmass_list[i].velocity.Y * inverse_count;

                force.X += pointmass_list[i].force.X * inverse_count;
                force.Y += pointmass_list[i].force.Y * inverse_count;
            }
        }

        public void UpdateBodyPositionVelocityForce(double elapsed)
        {
            GetBodyPositionVelocityForce(out position, out velocity, out force);
        }

        public void RotateShape(double elapsed)
        {
            // find the average angle of all of the masses.
            float angle = 0;
            int originalSign = 1;
            float originalAngle = 0;
            for (int i = 0; i < count; i++)
            {
                Vector2 baseNorm = new Vector2();
                baseNorm.X = base_shape.points[i].X;
                baseNorm.Y = base_shape.points[i].Y;
                Vector2.Normalize(ref baseNorm, out baseNorm);

                Vector2 curNorm = new Vector2();
                curNorm.X = pointmass_list[i].position.X - position.X;
                curNorm.Y = pointmass_list[i].position.Y - position.Y;
                Vector2.Normalize(ref curNorm, out curNorm);

                float dot;
                Vector2.Dot(ref baseNorm, ref curNorm, out dot);
                if (dot > 1.0f) { dot = 1.0f; }
                if (dot < -1.0f) { dot = -1.0f; }

                float thisAngle = (float)Math.Acos(dot);
                if (!VectorHelper.IsCCW(ref baseNorm, ref curNorm)) { thisAngle = -thisAngle; }

                if (i == 0)
                {
                    originalSign = (thisAngle >= 0.0f) ? 1 : -1;
                    originalAngle = thisAngle;
                }
                else
                {
                    float diff = (thisAngle - originalAngle);
                    int thisSign = (thisAngle >= 0.0f) ? 1 : -1;

                    if ((Math.Abs(diff) > Math.PI) && (thisSign != originalSign))
                    {
                        thisAngle = (thisSign == -1) ? ((float)Math.PI + ((float)Math.PI + thisAngle)) : (((float)Math.PI - thisAngle) - (float)Math.PI);
                    }
                }

                angle += thisAngle;
            }

            angle = angle / count;

            // now calculate the derived Omega, based on change in angle over time.
            float angleChange = (angle - prev_angle);
            if (Math.Abs(angleChange) >= Math.PI)
            {
                if (angleChange < 0f)
                    angleChange = angleChange + (float)(Math.PI * 2);
                else
                    angleChange = angleChange - (float)(Math.PI * 2);
            }

            omega = angleChange / (float)elapsed;
            prev_angle = angle;

            for (int i = 0; i < count; i++)
            {
                float x = base_shape.points[i].X * scale.X;
                float y = base_shape.points[i].Y * scale.Y;
                float c = (float)Math.Cos(angle);
                float s = (float)Math.Sin(angle);
                curr_shape.points[i].X = (c * x) - (s * y) + position.X;
                curr_shape.points[i].Y = (c * y) + (s * x) + position.Y;
            }
        }

        public virtual void ApplyInternalForces(double elapsed)
        { }

        public void Update(double elapsed)
        {
            if (!is_dirty)
                return;

            if (is_merging)
                return;

            SetBodyPositionVelocityForce(position, velocity, force);

            RotateShape(elapsed);
            
            ApplyInternalForces(elapsed);

            UpdatePointMasses(elapsed);
            UpdataAABB(elapsed);

            UpdateBodyPositionVelocityForce(elapsed);

            if (is_static)
                is_dirty = false;
        }

        public override string ToString()
        {
            return string.Format("{{position:[{0}] velocity:[{1}] force[{2}]}}", position, velocity, force);
        }

        public string ToStringSimple()
        {
            return string.Format("{{position:[{{{0:0.0}, {1:0.0}}}] velocity:[{{{2:0.0}, {3:0.0}}}]}}", position.X, position.Y, velocity.X, velocity.Y);
        }

        public bool Contains(ref Vector2 point)
        {
            // basic idea: draw a line from the point to a point known to be outside the body.  count the number of
            // lines in the polygon it intersects.  if that number is odd, we are inside.  if it's even, we are outside.
            // in this implementation we will always use a line that moves off in the positive X direction from the point
            // to simplify things.
            Vector2 endPt = new Vector2();
            endPt.X = aabb.max.X + 0.1f;
            endPt.Y = point.Y;

            // line we are testing against goes from pt -> endPt.
            bool inside = false;
            Vector2 edgeSt = pointmass_list[0].position;
            Vector2 edgeEnd = new Vector2();
            for (int i = 0; i < count; i++)
            {
                // the current edge is defined as the line from edgeSt -> edgeEnd.
                if (i < (count - 1))
                    edgeEnd = pointmass_list[i + 1].position;
                else
                    edgeEnd = pointmass_list[0].position;

                // perform check now...
                if (((edgeSt.Y <= point.Y) && (edgeEnd.Y > point.Y)) || ((edgeSt.Y > point.Y) && (edgeEnd.Y <= point.Y)))
                {
                    // this line crosses the test line at some point... does it do so within our test range?
                    float slope = (edgeEnd.X - edgeSt.X) / (edgeEnd.Y - edgeSt.Y);
                    float hitX = edgeSt.X + ((point.Y - edgeSt.Y) * slope);

                    if ((hitX >= point.X) && (hitX <= endPt.X))
                        inside = !inside;
                }
                edgeSt = edgeEnd;
            }

            return inside;
        }


        public float GetClosestPoint(Vector2 point, out Vector2 closest, out Vector2 normal, out int pointA, out int pointB, out float edgeD)
        {
            closest = Vector2.Zero;
            pointA = -1;
            pointB = -1;
            edgeD = 0f;
            normal = Vector2.Zero;

            float closestD = 1000.0f;

            for (int i = 0; i < count; i++)
            {
                Vector2 tempHit;
                Vector2 tempNorm;
                float tempEdgeD;

                float dist = GetClosestPointOnEdge(point, i, out tempHit, out tempNorm, out tempEdgeD);
                if (dist < closestD)
                {
                    closestD = dist;
                    pointA = i;
                    if (i < (count - 1))
                        pointB = i + 1;
                    else
                        pointB = 0;
                    edgeD = tempEdgeD;
                    normal = tempNorm;
                    closest = tempHit;
                }
            }


            // return.
            return closestD;
        }

        public float GetClosestPointOnEdge(Vector2 point, int edgeNum, out Vector2 hitPt, out Vector2 normal, out float edgeD)
        {
            hitPt = new Vector2();
            hitPt.X = 0f;
            hitPt.Y = 0f;

            normal = new Vector2();
            normal.X = 0f;
            normal.Y = 0f;

            edgeD = 0f;
            float dist = 0f;

            Vector2 ptA = pointmass_list[edgeNum].position;
            Vector2 ptB = new Vector2();

            if (edgeNum < (count - 1))
                ptB = pointmass_list[edgeNum + 1].position;
            else
                ptB = pointmass_list[0].position;

            Vector2 toP = new Vector2();
            toP.X = point.X - ptA.X;
            toP.Y = point.Y - ptA.Y;

            Vector2 E = new Vector2();
            E.X = ptB.X - ptA.X;
            E.Y = ptB.Y - ptA.Y;

            // get the length of the edge, and use that to normalize the vector.
            float edgeLength = (float)Math.Sqrt((E.X * E.X) + (E.Y * E.Y));
            if (edgeLength > 0.00001f)
            {
                E.X /= edgeLength;
                E.Y /= edgeLength;
            }

            // normal
            Vector2 n = new Vector2();
            VectorHelper.Perpendicular(ref E, ref n);

            // calculate the distance!
            float x;
            Vector2.Dot(ref toP, ref E, out x);
            if (x <= 0.0f)
            {
                // x is outside the line segment, distance is from pt to ptA.
                //dist = (pt - ptA).Length();
                Vector2.Distance(ref point, ref ptA, out dist);
                hitPt = ptA;
                edgeD = 0f;
                normal = n;
            }
            else if (x >= edgeLength)
            {
                // x is outside of the line segment, distance is from pt to ptB.
                //dist = (pt - ptB).Length();
                Vector2.Distance(ref point, ref ptB, out dist);
                hitPt = ptB;
                edgeD = 1f;
                normal = n;
            }
            else
            {
                // point lies somewhere on the line segment.
                Vector3 toP3 = new Vector3();
                toP3.X = toP.X;
                toP3.Y = toP.Y;

                Vector3 E3 = new Vector3();
                E3.X = E.X;
                E3.Y = E.Y;

                //dist = Math.Abs(Vector3.Cross(toP3, E3).Z);
                Vector3.Cross(ref toP3, ref E3, out E3);
                dist = Math.Abs(E3.Z);
                hitPt.X = ptA.X + (E.X * x);
                hitPt.Y = ptA.Y + (E.Y * x);
                edgeD = x / edgeLength;
                normal = n;
            }

            return dist;
        }

        public float GetClosestPointOnEdgeSquared(Vector2 point, int edgeNum, out Vector2 hitPt, out Vector2 normal, out float edgeD)
        {
            hitPt = new Vector2();
            hitPt.X = 0f;
            hitPt.Y = 0f;

            normal = new Vector2();
            normal.X = 0f;
            normal.Y = 0f;

            edgeD = 0f;
            float dist = 0f;

            Vector2 ptA = pointmass_list[edgeNum].position;
            Vector2 ptB = new Vector2();

            if (edgeNum < (count - 1))
                ptB = pointmass_list[edgeNum + 1].position;
            else
                ptB = pointmass_list[0].position;

            Vector2 toP = new Vector2();
            toP.X = point.X - ptA.X;
            toP.Y = point.Y - ptA.Y;

            Vector2 E = new Vector2();
            E.X = ptB.X - ptA.X;
            E.Y = ptB.Y - ptA.Y;

            // get the length of the edge, and use that to normalize the vector.
            float edgeLength = (float)Math.Sqrt((E.X * E.X) + (E.Y * E.Y));
            if (edgeLength > 0.00001f)
            {
                E.X /= edgeLength;
                E.Y /= edgeLength;
            }

            // normal
            Vector2 n = new Vector2();
            VectorHelper.Perpendicular(ref E, ref n);

            // calculate the distance!
            float x;
            Vector2.Dot(ref toP, ref E, out x);
            if (x <= 0.0f)
            {
                // x is outside the line segment, distance is from pt to ptA.
                //dist = (pt - ptA).Length();
                Vector2.DistanceSquared(ref point, ref ptA, out dist);
                hitPt = ptA;
                edgeD = 0f;
                normal = n;
            }
            else if (x >= edgeLength)
            {
                // x is outside of the line segment, distance is from pt to ptB.
                //dist = (pt - ptB).Length();
                Vector2.DistanceSquared(ref point, ref ptB, out dist);
                hitPt = ptB;
                edgeD = 1f;
                normal = n;
            }
            else
            {
                // point lies somewhere on the line segment.
                Vector3 toP3 = new Vector3();
                toP3.X = toP.X;
                toP3.Y = toP.Y;

                Vector3 E3 = new Vector3();
                E3.X = E.X;
                E3.Y = E.Y;

                //dist = Math.Abs(Vector3.Cross(toP3, E3).Z);
                Vector3.Cross(ref toP3, ref E3, out E3);
                dist = Math.Abs(E3.Z * E3.Z);
                hitPt.X = ptA.X + (E.X * x);
                hitPt.Y = ptA.Y + (E.Y * x);
                edgeD = x / edgeLength;
                normal = n;
            }

            return dist;
        }

        public PointMass GetClosestPointMass(Vector2 point, out float dist)
        {
            float closestSQD = 100000.0f;
            int closest = -1;

            for (int i = 0; i < count; i++)
            {
                float thisD = (point - pointmass_list[i].position).LengthSquared();
                if (thisD < closestSQD)
                {
                    closestSQD = thisD;
                    closest = i;
                }
            }

            dist = (float)Math.Sqrt(closestSQD);
            return pointmass_list[closest];
        }

        public void ApplyForce(ref Vector2 point, ref Vector2 force)
        {
            Vector2 R = (position - point);

            float torqueF = Vector3.Cross(VectorHelper.Vector3FromVector2(R), VectorHelper.Vector3FromVector2(force)).Z;

            for (int i = 0; i < count; i++)
            {
                Vector2 toPt = (pointmass_list[i].position - position);
                Vector2 torque = VectorHelper.Rotate(toPt, -(float)(Math.PI) / 2f);

                pointmass_list[i].force += torque * torqueF;
                pointmass_list[i].force += force;
            }
        }
    }
}
