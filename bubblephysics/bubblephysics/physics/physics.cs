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

    public class Physics
    {
        private static Random rand = new Random();
        public List<Chain> chain_list;
        public List<Body> body_list;
        public List<CollisionInfo> collision_list;
        public int penetration_count;
        public float penetration_threshold = 0.015f;
        public float friction = 1.9f;
        public float elasticity = 1.5f;

        public AxisAlignedBoundingBox aabb;
        public Vector2 size;
        public Vector2 cell;
        private bool initialized;

        public Action<Body, Body> on_aabb_collision;
        public Action<Body, Body, CollisionInfo> on_collision;
        public Action<float, Body, Body> on_penetration;

        public Physics()
        {
            this.chain_list = new List<Chain>();
            this.body_list = new List<Body>();
            this.collision_list = new List<CollisionInfo>();
            this.initialized = false;
        }

        public void Add(Chain chain)
        {
            if (!chain_list.Contains(chain))
                chain_list.Add(chain);
        }

        public void Remove(Chain chain)
        {
            if (chain_list.Contains(chain))
                chain_list.Remove(chain);
        }

        public void Add(Body body)
        {
            if (!body_list.Contains(body))
                body_list.Add(body);
        }

        public void Remove(Body body)
        {
            if (body_list.Contains(body))
                body_list.Remove(body);
        }

        public void SetWorldLimits(Vector2 min, Vector2 max)
        {
            aabb = new AxisAlignedBoundingBox(ref min, ref max);
            size = max - min;
            cell = size / 32;
        }

        public void UpdateBitmask(Body body)
        {
            AxisAlignedBoundingBox box = body.aabb;

            int minX = (int)Math.Floor((box.min.X - aabb.min.X) / cell.X);
            int maxX = (int)Math.Floor((box.max.X - aabb.min.X) / cell.X);

            if (minX < 0) { minX = 0; } else if (minX > 32) { minX = 32; }
            if (maxX < 0) { maxX = 0; } else if (maxX > 32) { maxX = 32; }

            int minY = (int)Math.Floor((box.min.Y - aabb.min.Y) / cell.Y);
            int maxY = (int)Math.Floor((box.max.Y - aabb.min.Y) / cell.Y);

            if (minY < 0) { minY = 0; } else if (minY > 32) { minY = 32; }
            if (maxY < 0) { maxY = 0; } else if (maxY > 32) { maxY = 32; }

            body.bitmaskx.clear();
            for (int i = minX; i <= maxX; i++)
                body.bitmaskx.setOn(i);

            body.bitmasky.clear();
            for (int i = minY; i <= maxY; i++)
                body.bitmasky.setOn(i);
        }

        public bool IsPointInsideAnyBody(Vector2 point)
        {
            for (int i = 0; i < body_list.Count; i++)
            {
                Body body = body_list[i];

                // broad-phase collision via AABB.
                if (!body.aabb.Contains(point.X,point.Y))
                    continue;

                if (body.Contains(ref point))
                    return true;
            }

            return false;
        }

        public void Initialize()
        {
            Vector2 min = Vector2.Zero;
            Vector2 max = Vector2.Zero;

            for (int i = 0; i < body_list.Count; i++)
            {
                if (!body_list[i].is_static)
                    continue;

                body_list[i].RotateShape(0);
                body_list[i].Update(0);

                if (body_list[i].aabb.min.X < min.X)
                    min.X = body_list[i].aabb.min.X;
                if (body_list[i].aabb.min.Y < min.Y)
                    min.Y = body_list[i].aabb.min.Y;

                if (body_list[i].aabb.max.X > max.X)
                    max.X = body_list[i].aabb.max.X;
                if (body_list[i].aabb.max.Y > max.Y)
                    max.Y = body_list[i].aabb.max.Y;
            }

            SetWorldLimits(min, max);
        }

        public void MoveDistantBodies(Vector2 position, float near, float far)
        {
            for (int i = 0; i < body_list.Count; i++)
            {
                Body body = body_list[i];

                if (body.is_static)
                    continue;

                float distance = (body.position - position).Length();

                if (distance > far)
                {
                    Vector2 point;
#if XBOX360
                    point = new Vector2();
#endif
                    point.X = ((float)rand.NextDouble() - 0.5f);
                    point.Y = ((float)rand.NextDouble() - 0.5f);
                    point.Normalize();

                    point *= near + (far - near) * ((float)rand.NextDouble());
                    point += position;

                    while (IsPointInsideAnyBody(point))
                    {
                        point.X = ((float)rand.NextDouble() - 0.5f);
                        point.Y = ((float)rand.NextDouble() - 0.5f);
                        point.Normalize();

                        point *= near + (far - near) * ((float)rand.NextDouble());
                        point += position;
                    }

                    body_list[i].position = point;
                    body_list[i].Update(0);
                }
            }
        }

        public void Update(double elapsed)
        {
            if (!initialized)
                Initialize();

            penetration_count = 0;
            collision_list.Clear();

            for (int i = 0; i < body_list.Count; i++)
            {
                body_list[i].Update(elapsed);
                UpdateBitmask(body_list[i]);
            }

            // update chains
            for (int i = 0; i < chain_list.Count; i++)
                chain_list[i].Update(elapsed);

            // now check for collision.
            // inter-body collision!
            for (int i = 0; i < body_list.Count; i++)
            {
                for (int j = i + 1; j < body_list.Count; j++)
                {
                    if (body_list[i].is_static && body_list[j].is_static)
                        continue;

                    // grid-based early out.
                    if (((body_list[i].bitmaskx.mask & body_list[j].bitmaskx.mask) == 0) &&
                        ((body_list[i].bitmasky.mask & body_list[j].bitmasky.mask) == 0))
                        continue;
                    
                    // broad-phase collision via AABB.
                    if (!(body_list[i].aabb).Intersects(ref (body_list[j].aabb)))
                        continue;

                    if (on_aabb_collision != null)
                        this.on_aabb_collision(body_list[i], body_list[j]);

                    // okay, the AABB's of these 2 are intersecting.  now check for collision of A against B.
                    collision_list.AddRange(Collision.Intersects(body_list[j], body_list[i]));
                    collision_list.AddRange(Collision.Intersects(body_list[i], body_list[j]));
                }
            }

            // now handle all collisions found during the update at once.
            // handle all collisions!
            for (int i = 0; i < collision_list.Count; i++)
            {
                CollisionInfo info = collision_list[i];

                PointMass A = info.pointmass_a;
                PointMass B1 = info.pointmass_b;
                PointMass B2 = info.pointmass_c;

                if (on_collision != null)
                    this.on_collision(info.body_a, info.body_b, info);

                // velocity changes as a result of collision.
                Vector2 bVel = new Vector2();
                bVel.X = (B1.velocity.X + B2.velocity.X) * 0.5f;
                bVel.Y = (B1.velocity.Y + B2.velocity.Y) * 0.5f;

                Vector2 relVel = new Vector2();
                relVel.X = A.velocity.X - bVel.X;
                relVel.Y = A.velocity.Y - bVel.Y;

                float relDot;
                Vector2.Dot(ref relVel, ref info.normal, out relDot);

                if (on_penetration != null)
                    this.on_penetration(info.penetration, info.body_a, info.body_b);

                if (info.penetration > 0.3f)
                {
                    penetration_count++;
                    continue;
                }

                float b1inf = 1.0f - info.edge_distance;
                float b2inf = info.edge_distance;

                float b2MassSum = ((float.IsPositiveInfinity(B1.mass)) || (float.IsPositiveInfinity(B2.mass))) ? float.PositiveInfinity : (B1.mass + B2.mass);

                float massSum = A.mass + b2MassSum;

                float Amove;
                float Bmove;
                if (float.IsPositiveInfinity(A.mass))
                {
                    Amove = 0f;
                    Bmove = (info.penetration) + 0.001f;
                }
                else if (float.IsPositiveInfinity(b2MassSum))
                {
                    Amove = (info.penetration) + 0.001f;
                    Bmove = 0f;
                }
                else
                {
                    Amove = (info.penetration * (b2MassSum / massSum));
                    Bmove = (info.penetration * (A.mass / massSum));
                }

                float B1move = Bmove * b1inf;
                float B2move = Bmove * b2inf;

                float AinvMass = (float.IsPositiveInfinity(A.mass)) ? 0f : 1f / A.mass;
                float BinvMass = (float.IsPositiveInfinity(b2MassSum)) ? 0f : 1f / b2MassSum;

                float jDenom = AinvMass + BinvMass;
                Vector2 numV = new Vector2();
                float elas = elasticity;
                numV.X = relVel.X * elas;
                numV.Y = relVel.Y * elas;

                float jNumerator;
                Vector2.Dot(ref numV, ref info.normal, out jNumerator);
                jNumerator = -jNumerator;

                float j = jNumerator / jDenom;

                if (!float.IsPositiveInfinity(A.mass))
                {
                    A.position.X += info.normal.X * Amove;
                    A.position.Y += info.normal.Y * Amove;
                }

                if (!float.IsPositiveInfinity(B1.mass))
                {
                    B1.position.X -= info.normal.X * B1move;
                    B1.position.Y -= info.normal.Y * B1move;
                }

                if (!float.IsPositiveInfinity(B2.mass))
                {
                    B2.position.X -= info.normal.X * B2move;
                    B2.position.Y -= info.normal.Y * B2move;
                }

                Vector2 tangent = new Vector2();
                VectorHelper.Perpendicular(ref info.normal, ref tangent);
                float fNumerator;
                Vector2.Dot(ref relVel, ref tangent, out fNumerator);
                fNumerator *= friction;
                float f = fNumerator / jDenom;

                // adjust velocity if relative velocity is moving toward each other.
                if (relDot <= 0.0001f)
                {
                    if (!float.IsPositiveInfinity(A.mass))
                    {
                        A.velocity.X += (info.normal.X * (j / A.mass)) - (tangent.X * (f / A.mass));
                        A.velocity.Y += (info.normal.Y * (j / A.mass)) - (tangent.Y * (f / A.mass));
                    }

                    if (!float.IsPositiveInfinity(b2MassSum))
                    {
                        B1.velocity.X -= (info.normal.X * (j / b2MassSum) * b1inf) - (tangent.X * (f / b2MassSum) * b1inf);
                        B1.velocity.Y -= (info.normal.Y * (j / b2MassSum) * b1inf) - (tangent.Y * (f / b2MassSum) * b1inf);
                    }

                    if (!float.IsPositiveInfinity(b2MassSum))
                    {
                        B2.velocity.X -= (info.normal.X * (j / b2MassSum) * b2inf) - (tangent.X * (f / b2MassSum) * b2inf);
                        B2.velocity.Y -= (info.normal.Y * (j / b2MassSum) * b2inf) - (tangent.Y * (f / b2MassSum) * b2inf);
                    }

                    //info.body_a.UpdateBodyPositionVelocityForce(elapsed);
                   // info.body_b.UpdateBodyPositionVelocityForce(elapsed);
                }
            }

            for (int i = 0; i < body_list.Count; i++)
                body_list[i].UpdateBodyPositionVelocityForce(elapsed);
        }
    }
}
