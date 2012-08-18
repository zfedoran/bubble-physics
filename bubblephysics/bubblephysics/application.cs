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
using Microsoft.Xna.Framework.Input;

namespace bubblephysics
{
    public class Application : Microsoft.Xna.Framework.Game
    {
        private BubblePhysics game;

        public Application()
        { this.game = new BubblePhysics(this); }

        protected override void Initialize()
        { base.Initialize(); }

        protected override void LoadContent()
        { this.game.Initialize(); }

        protected override void UnloadContent()
        { this.game.Uninitialize(); }

        protected override void Update(GameTime gameTime)
        {
            this.game.Update((float)gameTime.ElapsedGameTime.TotalSeconds);
            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            this.game.Render();
            base.Draw(gameTime);
        }
    }
}
