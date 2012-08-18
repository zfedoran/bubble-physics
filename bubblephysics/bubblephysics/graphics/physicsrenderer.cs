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
using Microsoft.Xna.Framework.Graphics;
using bubblephysics.physics;
using bubblephysics.core;

namespace bubblephysics.graphics
{
    public class PhysicsRenderer
    {
        private GraphicsDevice device;
        private LineBatch linebatch;
        private BasicEffect basiceffect;
        private Camera camera;
        private Physics physics;
        
        public PhysicsRenderer(GraphicsDevice device)
        {
            this.device = device;
            this.linebatch = new LineBatch(device);
            this.basiceffect = new BasicEffect(device);
            this.basiceffect.VertexColorEnabled = true;
        }

        public void SetCamera(Camera camera)
        {
            this.camera = camera;
        }

        public void SetPhysics(Physics physics)
        {
            this.physics = physics;
        }

        public void Render()
        {
            basiceffect.World = Matrix.Identity;
            basiceffect.View = camera.GetViewMatrix();
            basiceffect.Projection = camera.GetProjectionMatrix();
            basiceffect.CurrentTechnique.Passes[0].Apply();

            linebatch.Begin();
                for (int i = 0; i < physics.body_list.Count; i++)
                {
                    RenderBody(physics.body_list[i]);
                }
                for (int i = 0; i < physics.chain_list.Count; i++)
                {
                    RenderChain(physics.chain_list[i]);
                }
            linebatch.End();
        }

        public void RenderChain(Chain chain)
        {
            List<PointMass> points = chain.pointmass_list;
            Color color = Color.Green;
            for (int i = 0; i < points.Count-1; i++)
            {
                linebatch.Add(new LineVertex(new Vector3(points[i].position.X, points[i].position.Y, 0), color));
                linebatch.Add(new LineVertex(new Vector3(points[i + 1].position.X, points[i + 1].position.Y, 0), color));
            }
        }

        public void RenderBody(Body body)
        {
           // RenderAABB(body.aabb);
            //RenderShape(body.curr_shape);
            RenderOutlines(body.pointmass_list);
           // RenderNormals(body.pointmass_list);
        }

        public void RenderAABB(AxisAlignedBoundingBox aabb)
        {
            Color color = new Color(1, 1, 1, 0.3f);
            float x, y, w, h;
            x = aabb.min.X;
            y = aabb.min.Y;
            w = aabb.max.X - x;
            h = aabb.max.Y - y;

            linebatch.Add(new LineVertex(new Vector3(x + w * 0, y + h * 0, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 0, y + h * 1, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 0, y + h * 1, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 1, y + h * 1, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 1, y + h * 1, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 1, y + h * 0, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 1, y + h * 0, 0), color));
            linebatch.Add(new LineVertex(new Vector3(x + w * 0, y + h * 0, 0), color));
        }

        public void RenderNormals(PointMass[] points)
        {
            Color color = Color.Purple;
            float size = 0.1f;

            int prevPt, nextPt;
            Vector2 pt, prev, next, fromPrev, toNext, ptNorm;
            fromPrev = toNext = ptNorm = new Vector2();

            for (int i = 0; i < points.Length; i++)
            {
                pt = points[i].position;
                prevPt = (i > 0) ? i - 1 : points.Length - 1;
                nextPt = (i < points.Length - 1) ? i + 1 : 0;
                prev = points[prevPt].position;
                next = points[nextPt].position;
                fromPrev.X = pt.X - prev.X;
                fromPrev.Y = pt.Y - prev.Y;
                toNext.X = next.X - pt.X;
                toNext.Y = next.Y - pt.Y;
                fromPrev.Normalize();
                toNext.Normalize();
                ptNorm.X = -(fromPrev.Y + toNext.Y);
                ptNorm.Y = fromPrev.X + toNext.X;
                ptNorm.Normalize();

                linebatch.Add(new LineVertex(new Vector3(pt.X, pt.Y, 0), color));
                linebatch.Add(new LineVertex(new Vector3(pt.X + ptNorm.X * size, pt.Y + ptNorm.Y * size, 0), color));
            }
        }

        public void RenderOutlines(PointMass[] points)
        {
            Color color = Color.White;

            int i;
            for (i = 0; i < points.Length - 1; i++)
            {
                linebatch.Add(new LineVertex(new Vector3(points[i].position.X, points[i].position.Y, 0), color));
                linebatch.Add(new LineVertex(new Vector3(points[i + 1].position.X, points[i + 1].position.Y, 0), color));
            }
            linebatch.Add(new LineVertex(new Vector3(points[i].position.X, points[i].position.Y, 0), color));
            linebatch.Add(new LineVertex(new Vector3(points[0].position.X, points[0].position.Y, 0), color));
        }

        public void RenderShape(Shape shape)
        {
            Color color = new Color(0.5f, 0, 0.5f, 0.3f);

            int i;
            for (i = 0; i < shape.points.Length - 1; i++)
            {
                linebatch.Add(new LineVertex(new Vector3(shape.points[i].X, shape.points[i].Y, 0), color));
                linebatch.Add(new LineVertex(new Vector3(shape.points[i + 1].X, shape.points[i + 1].Y, 0), color));
            }
            linebatch.Add(new LineVertex(new Vector3(shape.points[i].X, shape.points[i].Y, 0), color));
            linebatch.Add(new LineVertex(new Vector3(shape.points[0].X, shape.points[0].Y, 0), color));
        }
    }
}
