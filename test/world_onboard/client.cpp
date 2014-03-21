#include <cstdlib>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "my_world.hpp"

static const int WIN_WIDTH = 800;
static const int WIN_HEIGHT = 600;

static const int RELATION = 4;
static const int ORIGIN_X = 400;
static const int ORIGIN_Y = 325;

static const int POINT_SIZE = 3;

static const sf::Color WORLD_COLOR = sf::Color::White;
static const sf::Color MOVABLE_COLOR = sf::Color::Red;

static sf::RenderWindow* win = NULL;
static sf::Color col;

void draw_point(const Point& p) {
  int x = ORIGIN_X + 1.f * p[0] / RELATION;
  int y = ORIGIN_Y - 1.f * p[1] / RELATION;
  
  sf::Shape sh = sf::Shape::Line(x - POINT_SIZE, y + POINT_SIZE, x + POINT_SIZE, y - POINT_SIZE, 1.f, col);
  win->Draw(sh);
  sh = sf::Shape::Line(x - POINT_SIZE, y - POINT_SIZE, x + POINT_SIZE, y + POINT_SIZE, 1.f, col);
  win->Draw(sh);
}

void draw_segment(const Segment& s) {
  sf::Shape sh =
    sf::Shape::Line(ORIGIN_X + 1.f * s[0][0] / RELATION,
		    ORIGIN_Y - 1.f * s[0][1] / RELATION,
		    ORIGIN_X + 1.f * s[1][0] / RELATION,
		    ORIGIN_Y - 1.f * s[1][1] / RELATION,
		    1.f, col);
  win->Draw(sh);
}

void draw_circle(const Circle& c) {
  sf::Shape sh =
    sf::Shape::Circle(ORIGIN_X + 1.f * c.centre()[0] / RELATION,
		      ORIGIN_Y - 1.f * c.centre()[1] / RELATION,
		      1.f * c.radius() / RELATION,
		      col);
  win->Draw(sh);
}

void draw_AABB(const AABB& a) {
  sf::Shape sh =
    sf::Shape::Rectangle(ORIGIN_X + 1.f * a.o()[0] / RELATION,
			 ORIGIN_Y - 1.f * a.o()[1] / RELATION,
			 ORIGIN_X + 1.f * (a.o()[0] + (s32) a.w()) / RELATION,
			 ORIGIN_Y - 1.f * (a.o()[1] + (s32) a.h()) / RELATION,
			 col);
  win->Draw(sh);
}

void draw_shape(const Shape* s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(s))) {
    draw_point(*static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(s))) {
    draw_segment(*static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(s))) {
    draw_circle(*static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const AABB*>(s))) {
    draw_AABB(*static_cast<const AABB*>(ptr));
  }
}

void draw_world(const World<WORLD_SIZE>& world) {
  col = WORLD_COLOR;
  world.doForeach(draw_shape);
}

void draw_movable(void) {
  col = MOVABLE_COLOR;
  draw_shape(&red_top_fire);
  draw_shape(&red_mid_fire);
  draw_shape(&red_bot_fire);
  draw_shape(&red_wall_fire);
  
  draw_shape(&yellow_top_fire);
  draw_shape(&yellow_mid_fire);
  draw_shape(&yellow_bot_fire);
  draw_shape(&yellow_wall_fire);
  
  draw_shape(&red_movable_torch);
  draw_shape(&yellow_movable_torch);
}

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  
  sf::RenderWindow app(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "2014 world - Eirbot");
  win = &app;
  
  sf::String title("2014 world - Eirbot");
  title.SetPosition((WIN_WIDTH - title.GetRect().GetWidth()) / 2, (50 - title.GetRect().GetHeight()) / 2);
  
  World<WORLD_SIZE> world;
  fill_world(world);
  
  while(app.IsOpened())
    {
      sf::Event event;
      while(app.GetEvent(event))
	{
	  if(event.Type == sf::Event::Closed) {
	    app.Close();
	  }
	}
      
      if(!app.IsOpened()) {
	break;
      }
      
      app.Clear();
      
      app.Draw(title);
      draw_world(world);
      draw_movable();
      app.Display();
      
      sf::Sleep(0.050f);
    }
  return EXIT_SUCCESS;
}
