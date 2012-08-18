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
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using bubblephysics.graphics;
using bubblephysics.core;
using bubblephysics.physics;
using bubblephysics.time;

namespace bubblephysics
{
    public class BubblePhysics
    {
        private static int WIDTH = 1280;
        private static int HEIGHT = 720;
        private static Color CLEAR_COLOR = new Color(0.3f, 0.3f, 0.3f, 1);

        private Application app;
        private GraphicsDeviceManager graphics;
        private GraphicsDevice device;
        private ContentManager content;

        private BasicEffect basiceffect;
        private SpriteBatch spritebatch;
        private TriangleBatch trianglebatch;
        private LineBatch linebatch;
        private SpriteFont debugFont;

        private Camera camera;
        private float time;
        private float fps;

        private Physics physics;
        private PhysicsRenderer physicsRenderer;
        private PhysicsStateManager physicsStateManager;
        private SpringBody ground;
        private PressureBody bubble;
        private bool rewinding;
        
        public BubblePhysics(Application app)
        {
            this.app = app;
            graphics = new GraphicsDeviceManager(app);
            content = app.Content;

            app.IsFixedTimeStep = true;
            graphics.SynchronizeWithVerticalRetrace = true;
            graphics.PreferredBackBufferWidth = WIDTH;
            graphics.PreferredBackBufferHeight = HEIGHT;
            content.RootDirectory = "Content";
        }

        public void Initialize()
        {
            device = this.graphics.GraphicsDevice;

            camera = new Camera(0.1f, 100, 1.2f, WIDTH / (float)HEIGHT);
            camera.position = new Vector3(0, 0, 0);
            camera.target = new Vector3(0, 0, 0);

            trianglebatch = new TriangleBatch(device);
            linebatch = new LineBatch(device);
            spritebatch = new SpriteBatch(device);
            basiceffect = new BasicEffect(device);
            basiceffect.VertexColorEnabled = true;
            
            debugFont = content.Load<SpriteFont>("fonts/debug");

            physics = new Physics();
            physicsRenderer = new PhysicsRenderer(device);
            physicsRenderer.SetPhysics(physics);
            physicsRenderer.SetCamera(camera);
            physicsStateManager = new PhysicsStateManager(physics);
            physicsStateManager.SetRecordingFrequency(1 / 15f);

            AddPhysicsBodies();
        }

        public void AddPhysicsBodies()
        {
            Shape groundShape = new Shape();
            groundShape.Begin(true);
            groundShape.Add(new Vector2(-15, 0));
            groundShape.Add(new Vector2(-15, 1));
            groundShape.Add(new Vector2(15, 1));
            groundShape.Add(new Vector2(15, 0));
            groundShape.End();

            ground = new SpringBody(groundShape, float.PositiveInfinity, 0, 0, 0, 0);
            ground.is_static = true;
            ground.position.Y = -1;
            physics.Add(ground);

            float scale = 0.5f;
            Shape boxShape = new Shape();
            boxShape.Begin(true);
            boxShape.Add(new Vector2(0, 0));
            boxShape.Add(new Vector2(0, scale * 0.5f));
            boxShape.Add(new Vector2(scale * 0.5f, scale * 0.5f));
            boxShape.Add(new Vector2(scale * 0.5f, 0));
            boxShape.End();

            for (int i = 0; i < 50; i++)
            {
                bubble = new PressureBody(boxShape, 1, 1, 1300, 20, 1000, 20);
                bubble.position.Y = i * scale * 0.5f;
                bubble.position.X = 0;
                physics.Add(bubble);
            }


            Shape circleShape = new Shape();
            circleShape.Begin(true);
            for (int i = 320; i >= 0; i -= 40)
            {
                circleShape.Add(new Vector2((float)Math.Cos(i / 180f * Math.PI) * 0.25f, (float)Math.Sin(i / 180f * Math.PI) * 0.25f));
            }
            circleShape.End();

            bubble = new PressureBody(circleShape, 1, 1, 130, 20, 130, 20);
            bubble.position.Y = 5;
            bubble.position.X = 5;
            physics.Add(bubble);
        }

        public void Uninitialize()
        {

        }

        public void Update(float elapsed)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                app.Exit();
            if (Keyboard.GetState().IsKeyDown(Keys.Escape))
                app.Exit();

            if (rewinding)
            {
                time -= elapsed;
                physicsStateManager.RewindTo(time);
            }
            else
            {
                time += elapsed;
                physicsStateManager.Save(time);
                physics.Update(elapsed);
            }
            
            fps = 1/elapsed;

            for (int i = 0; i < physics.body_list.Count; i++)
            {
                if (!physics.body_list[i].is_static)
                    physics.body_list[i].velocity.Y += -1 * elapsed;
            }

            camera.position.X = bubble.position.X;
            camera.position.Y = bubble.position.Y;
            camera.position.Z = 8;

            camera.target.X = bubble.position.X;
            camera.target.Y = bubble.position.Y;

            bubble.velocity += 2 * elapsed * GamePad.GetState(PlayerIndex.One).ThumbSticks.Left;

            if (Keyboard.GetState().IsKeyDown(Keys.W))
                bubble.velocity.Y += 4 * elapsed;
            if (Keyboard.GetState().IsKeyDown(Keys.S))
                bubble.velocity.Y -= 4 * elapsed;
            if (Keyboard.GetState().IsKeyDown(Keys.D))
                bubble.velocity.X += 4 * elapsed;
            if (Keyboard.GetState().IsKeyDown(Keys.A))
                bubble.velocity.X -= 4 * elapsed;
#if XBOX360
            if (GamePad.GetState(PlayerIndex.One).Buttons.A == ButtonState.Pressed)
                this.rewinding = true;
            if (GamePad.GetState(PlayerIndex.One).Buttons.A == ButtonState.Released)
                this.rewinding = false;
#else 
            if (Keyboard.GetState().IsKeyDown(Keys.R))
                this.rewinding = true;
            if (Keyboard.GetState().IsKeyUp(Keys.R))
                this.rewinding = false;
#endif
        }

        public void Render()
        {
            device.BlendState = BlendState.NonPremultiplied;
            device.RasterizerState = RasterizerState.CullCounterClockwise;
            device.DepthStencilState = DepthStencilState.Default;
            if(!rewinding) device.Clear(CLEAR_COLOR);

            basiceffect.World = Matrix.Identity;
            basiceffect.View = camera.GetViewMatrix();
            basiceffect.Projection = camera.GetProjectionMatrix();
            basiceffect.CurrentTechnique.Passes[0].Apply();

            linebatch.Begin();
                RenderAxis(linebatch);
            linebatch.End();

            physicsRenderer.Render();

            RenderText(10, 10, Color.White, string.Format("{0:0.00} fps", fps));
            RenderText(10, 30, Color.White, string.Format("{0:0.0000} seconds", time));
            RenderText(10, 50, Color.White, string.Format("{0:0} current index", physicsStateManager.GetCurrentIndex(bubble)));

            string helpText;
#if XBOX360
            helpText = "press and hold '(A)' to rewind time";
#else
            helpText = "press and hold 'R' to rewind time";
#endif
            RenderText((int)(WIDTH / 2 - debugFont.MeasureString(helpText).X / 2), 50, Color.White, helpText);
        }

        public void RenderText(float x, float y, Color color, string text)
        {
            Vector2 textScale = debugFont.MeasureString(text);

            float w, h;
            w = textScale.X;
            h = textScale.Y;
            float pt, pb, pl, pr;
            pt = -2; pb = 0; pl = -5; pr = 5;
            Color background = new Color(0, 0, 0, 0.3f);

            basiceffect.View = Matrix.Identity;
            basiceffect.Projection = Matrix.CreateOrthographicOffCenter(0, WIDTH, HEIGHT, 0, camera.near, camera.far);
            basiceffect.CurrentTechnique.Passes[0].Apply();

            trianglebatch.Begin();
                trianglebatch.Add(new TriangleVertex(new Vector3(x + 0 * w + pl, y + 0 * h + pt, -1), Vector3.Zero, Vector2.Zero, background));
                trianglebatch.Add(new TriangleVertex(new Vector3(x + 1 * w + pr, y + 1 * h + pb, -1), Vector3.Zero, Vector2.Zero, background));
                trianglebatch.Add(new TriangleVertex(new Vector3(x + 0 * w + pl, y + 1 * h + pb, -1), Vector3.Zero, Vector2.Zero, background));

                trianglebatch.Add(new TriangleVertex(new Vector3(x + 0 * w + pl, y + 0 * h + pt, -1), Vector3.Zero, Vector2.Zero, background));
                trianglebatch.Add(new TriangleVertex(new Vector3(x + 1 * w + pr, y + 0 * h + pt, -1), Vector3.Zero, Vector2.Zero, background));
                trianglebatch.Add(new TriangleVertex(new Vector3(x + 1 * w + pr, y + 1 * h + pb, -1), Vector3.Zero, Vector2.Zero, background));
            trianglebatch.End();

            spritebatch.Begin();
                spritebatch.DrawString(debugFont, text, new Vector2(x, y), color);
            spritebatch.End();

        }

        public void RenderAxis(LineBatch linebatch)
        {
            int size = 15;
            int offset = 20;

            basiceffect.View = Matrix.Identity;
            basiceffect.Projection = Matrix.CreateOrthographicOffCenter(-offset, WIDTH - offset, -offset, HEIGHT - offset, camera.near, camera.far);
            basiceffect.CurrentTechnique.Passes[0].Apply();

            linebatch.Add(new LineVertex(new Vector3(0, 0, -1), Color.Red));
            linebatch.Add(new LineVertex(new Vector3(size, 0, -1), Color.Red));

            linebatch.Add(new LineVertex(new Vector3(0, 0, -1), Color.Green));
            linebatch.Add(new LineVertex(new Vector3(0, size, -1), Color.Green));

            linebatch.Add(new LineVertex(new Vector3(0, 0, -1), Color.Blue));
            linebatch.Add(new LineVertex(new Vector3(0, 0, size-1), Color.Blue));
        }
    }
}
