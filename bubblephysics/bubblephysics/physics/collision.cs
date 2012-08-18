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
    public struct CollisionInfo
    {
        public Body body_a;
        public Body body_b;
        public PointMass pointmass_a;
        public PointMass pointmass_b;
        public PointMass pointmass_c;
        public float edge_distance;
        public Vector2 normal;
        public Vector2 point;
        public float penetration;

        public void Clear()
        {
            body_a = body_b = null;
            pointmass_a = pointmass_b = pointmass_c = new PointMass();
            edge_distance = 0f;
            point = Vector2.Zero;
            normal = Vector2.Zero;
            penetration = 0f;
        }
    }


    public class Collision
    {
        public static List<CollisionInfo> Intersects(Body body_a, Body body_b)
        {
            List<CollisionInfo> data = new List<CollisionInfo>();

            int bApmCount = body_a.count;
            int bBpmCount = body_b.count;

            AxisAlignedBoundingBox boxB = body_b.aabb;

            // check all PointMasses on bodyA for collision against bodyB.  if there is a collision, return detailed info.
            CollisionInfo infoAway = new CollisionInfo();
            CollisionInfo infoSame = new CollisionInfo();
            for (int i = 0; i < bApmCount; i++)
            {
                Vector2 pt = body_a.pointmass_list[i].position;

                // early out - if this point is outside the bounding box for bodyB, skip it!
                if (!boxB.Contains(pt.X, pt.Y))
                    continue;

                // early out - if this point is not inside bodyB, skip it!
                if (!body_b.Contains(ref pt))
                    continue;

                int prevPt = (i > 0) ? i - 1 : bApmCount - 1;
                int nextPt = (i < bApmCount - 1) ? i + 1 : 0;

                Vector2 prev = body_a.pointmass_list[prevPt].position;
                Vector2 next = body_a.pointmass_list[nextPt].position;

                // now get the normal for this point. (NOT A UNIT VECTOR)
                Vector2 fromPrev = new Vector2();
                fromPrev.X = pt.X - prev.X;
                fromPrev.Y = pt.Y - prev.Y;

                Vector2 toNext = new Vector2();
                toNext.X = next.X - pt.X;
                toNext.Y = next.Y - pt.Y;

                Vector2 ptNorm = new Vector2();
                ptNorm.X = fromPrev.X + toNext.X;
                ptNorm.Y = fromPrev.Y + toNext.Y;
                VectorHelper.Perpendicular(ref ptNorm);

                // this point is inside the other body.  now check if the edges on either side intersect with and edges on bodyB.          
                float closestAway = 100000.0f;
                float closestSame = 100000.0f;

                infoAway.Clear();
                infoAway.body_a = body_a;
                infoAway.pointmass_a = body_a.pointmass_list[i];
                infoAway.body_b = body_b;

                infoSame.Clear();
                infoSame.body_a = body_a;
                infoSame.pointmass_a = body_a.pointmass_list[i];
                infoSame.body_b = body_b;

                bool found = false;

                int b1 = 0;
                int b2 = 1;
                for (int j = 0; j < bBpmCount; j++)
                {
                    Vector2 hitPt;
                    Vector2 norm;
                    float edgeD;

                    b1 = j;

                    if (j < bBpmCount - 1)
                        b2 = j + 1;
                    else
                        b2 = 0;

                    Vector2 pt1 = body_b.pointmass_list[b1].position;
                    Vector2 pt2 = body_b.pointmass_list[b2].position;

                    // quick test of distance to each point on the edge, if both are greater than current mins, we can skip!
                    float distToA = ((pt1.X - pt.X) * (pt1.X - pt.X)) + ((pt1.Y - pt.Y) * (pt1.Y - pt.Y));
                    float distToB = ((pt2.X - pt.X) * (pt2.X - pt.X)) + ((pt2.Y - pt.Y) * (pt2.Y - pt.Y));


                    if ((distToA > closestAway) && (distToA > closestSame) && (distToB > closestAway) && (distToB > closestSame))
                        continue;

                    // test against this edge.
                    float dist = body_b.GetClosestPointOnEdgeSquared(pt, j, out hitPt, out norm, out edgeD);

                    // only perform the check if the normal for this edge is facing AWAY from the point normal.
                    float dot;
                    Vector2.Dot(ref ptNorm, ref norm, out dot);
                    if (dot <= 0f)
                    {
                        if (dist < closestAway)
                        {
                            closestAway = dist;
                            infoAway.pointmass_b = body_b.pointmass_list[b1];
                            infoAway.pointmass_c = body_b.pointmass_list[b2];
                            infoAway.edge_distance = edgeD;
                            infoAway.point = hitPt;
                            infoAway.normal = norm;
                            infoAway.penetration = dist;
                            found = true;
                        }
                    }
                    else
                    {
                        if (dist < closestSame)
                        {
                            closestSame = dist;
                            infoSame.pointmass_b = body_b.pointmass_list[b1];
                            infoSame.pointmass_c = body_b.pointmass_list[b2];
                            infoSame.edge_distance = edgeD;
                            infoSame.point = hitPt;
                            infoSame.normal = norm;
                            infoSame.penetration = dist;
                        }
                    }
                }

                // we've checked all edges on BodyB.  
                if ((found) && (closestAway > 0.3f) && (closestSame < closestAway))
                {
                    infoSame.penetration = (float)Math.Sqrt(infoSame.penetration);
                    data.Add(infoSame);
                }
                else
                {
                    infoAway.penetration = (float)Math.Sqrt(infoAway.penetration);
                    data.Add(infoAway);
                }
            }

            return data;
        }


    }

}
