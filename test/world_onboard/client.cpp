#include <cstdlib>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "my_world.hpp"
#include <geometry/collision.hpp>
#include <cstring>
#include <sstream>

#include "com.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sched.h>

static const int WIN_WIDTH = 1000;
static const int WIN_HEIGHT = 600;

static const int RELATION = 4;
static const int ORIGIN_X = 400;
static const int ORIGIN_Y = 325;

static const int POINT_SIZE = 3;

static const sf::Color WORLD_COLOR = sf::Color::White;
static const sf::Color MOVABLE_COLOR = sf::Color::Red;
static const sf::Color DEST_COLOR = sf::Color::Magenta;
static const sf::Color POS_COLOR = sf::Color::Green;

static sf::RenderWindow* win = NULL;
static sf::Color col;

static const char* DEVICE = "/dev/ttyACM0";

std::string int2str(int number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

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
  if(s->shapeId() == Point::ID) {
    draw_point(*reinterpret_cast<const Point*>(s));
  }
  else if(s->shapeId() == Segment::ID) {
    draw_segment(*reinterpret_cast<const Segment*>(s));
  }
  else if(s->shapeId() == Circle::ID) {
    draw_circle(*reinterpret_cast<const Circle*>(s));
  }
  else if(s->shapeId() == AABB::ID) {
    draw_AABB(*reinterpret_cast<const AABB*>(s));
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
  
  draw_shape(&left_wall_fire);
  draw_shape(&right_wall_fire);
  
  draw_shape(&red_movable_torch);
  draw_shape(&yellow_movable_torch);
}

void sendDest(int fd, const Point& dest) {
  write(fd, &dest.x(), 4);
  write(fd, &dest.y(), 4);
}

bool recvPos(int fd, s32& x, s32& y, int& n) {
  unsigned char c;
  int r;
  while((r = read(fd, &c, 1))) {
    if(n >= 0 && n <= 3) { // We are reading x
      x += c << (n * 8);
    }
    else { // We are reading y
      y += c << ((n - 4) * 8);
    }
    n += r;
    if(n == 8) { // we are done reading (x,y)
      break;
    }
  }
  return n == 8;
}

void synchronize(int fd) {
  int n = 0, i;
  char c;
  // Let's synchronize
  while(n < 4) {
    i = 0;
    while(!read(fd, &c, 1)) {
      if((++i) == 8) {
	// I got nothing to read for a while, let's wake up the board
	c = 'G';
	write(fd, &c, 1);
	sched_yield();
      }
    }
    if(c == n) {
      n++;
    }
    else {
      n = 0;
    }
  }
  // I received 0 1 2 3 correctly
  c = 'K';
  write(fd, &c, 1);
  // Waiting for synchro confirmation
  do {
    while(!read(fd, &c, 1)) {
      sched_yield();
    }
  } while(c != 'K');
  // We are synchronized
}

// Function from http://stackoverflow.com/questions/3407012#answer-3407254
int roundUp(int numToRound, int multiple) 
{ 
  if(multiple == 0) {
    return numToRound;
  } 
  
  int remainder = abs(numToRound) % multiple;
  if(remainder == 0) {
    return numToRound;
  }
  if(numToRound < 0) {
    return -(abs(numToRound) - remainder);
  }
  return numToRound + multiple - remainder;
}

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  
  sf::RenderWindow app(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "2014 world - Eirbot");
  win = &app;
  
  sf::String title("2014 world - Eirbot");
  title.SetPosition((WIN_WIDTH - 200 - title.GetRect().GetWidth()) / 2, (50 - title.GetRect().GetHeight()) / 2);
  
  sf::String start("Start");
  start.SetPosition(WIN_WIDTH - 200 + (200- start.GetRect().GetWidth()) / 2, (50 - title.GetRect().GetHeight()) / 2);
  sf::String stop("Stop");
  stop.SetPosition(WIN_WIDTH - 200 + (200 - stop.GetRect().GetWidth()) / 2, (50 - title.GetRect().GetHeight()) / 2);
  sf::String wait("Please wait");
  wait.SetPosition(WIN_WIDTH - 200 + (200 - wait.GetRect().GetWidth()) / 2, (50 - title.GetRect().GetHeight()) / 2);
  
  sf::String obs("OBSTACLE!!!");
  obs.SetColor(sf::Color::Red);
  obs.SetSize(96);
  obs.SetStyle(sf::String::Bold);
  obs.SetPosition((WIN_WIDTH - 200 - obs.GetRect().GetWidth()) / 2,
		  (WIN_HEIGHT - obs.GetRect().GetHeight()) / 2);
  
  sf::Shape obs_box =
    sf::Shape::Rectangle(obs.GetRect().Left, obs.GetRect().Top,
			 obs.GetRect().Right, obs.GetRect().Bottom,
			 sf::Color::Black, 3.f, sf::Color::Red);
  
  bool hw_on = false;
  World<WORLD_SIZE> world;
  fill_world(world);
  
  AABB button(WIN_WIDTH - 200, 0, 200, 50);
  Point dest(0, 500);
  sf::String dest_str("dest");
  dest_str.SetColor(DEST_COLOR);
  dest_str.SetPosition(WIN_WIDTH - 190, ((50 - title.GetRect().GetHeight()) / 2) * 6);
  Point pos(0, 500);
  sf::String pos_str("pos");
  pos_str.SetColor(POS_COLOR);
  pos_str.SetPosition(WIN_WIDTH - 190, ((50 - title.GetRect().GetHeight()) / 2) * 10);
  
  int fd = -1;
  
  int n;
  s32 x, y;
  
  int warning = 0;
  
  while(app.IsOpened()) {
    sf::Event event;
    while(app.GetEvent(event)) {
      if(event.Type == sf::Event::Closed) {
	app.Close();
      }
      else if(event.Type == sf::Event::MouseButtonReleased) {
	// If the user clicked on the left side
	if(hw_on && event.MouseButton.X < (WIN_WIDTH - 200)) {
	  Point tmp(roundUp((event.MouseButton.X - ORIGIN_X) * 4, 5), roundUp(-(event.MouseButton.Y - ORIGIN_Y) * 4, 5));
	  AABB inside_table(-1500, -950, 3000, 2000);
	  if(CollisionDetector::collide(tmp, inside_table)) {
	    if(!world.collide(tmp)) { // If tmp is not on an obstacle
	      dest = tmp;
	    }
	    else {
	      warning = 10;
	    }
	  }
	}
	// If the user clicked on the button
	else if(CollisionDetector::collide(Point(event.MouseButton.X, event.MouseButton.Y), button)) {
	  hw_on = !hw_on;
	  if(hw_on) {
	    n = x = y = 0;
	    if((fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC)) < 0) {
	      hw_on = false; // Could not open the device
	    }
	    else {
	      if(set_interface_attribs(fd, B9600, 0) < 0) {
		hw_on = false; // Could not set it up correctly
		close(fd);
		fd = -1;
	      }
	      else {
		set_blocking(fd, 0);
		
		app.Draw(sf::Shape::Rectangle(WIN_WIDTH - 200, 0,
					      WIN_WIDTH, WIN_HEIGHT,
					      sf::Color::Black));
		
		app.Draw(sf::Shape::Rectangle(WIN_WIDTH - 200, 0,
					      WIN_WIDTH - 198, WIN_HEIGHT,
					      sf::Color::Red));
		
		app.Draw(wait);
		app.Display();
		
		synchronize(fd);
		sendDest(fd, dest);
	      }
	    }
	  }
	  else {
	    close(fd);
	    fd = -1;
	  }
	}
      }
    }
    
    if(!app.IsOpened()) {
      break;
    }
    
    if(hw_on) {
      if(recvPos(fd, x, y, n)) {
	pos.x() = x;
	pos.y() = y;
	n = x = y = 0;
	sendDest(fd, dest);
      }
    }
    
    app.Clear();
    
    // Title display
    app.Draw(title);
    
    // World and movable objects display
    draw_world(world);
    draw_movable();
    
    // Right panel display
    app.Draw(sf::Shape::Rectangle(WIN_WIDTH - 200, 0,
				  WIN_WIDTH, WIN_HEIGHT,
				  sf::Color::Black));
    app.Draw(sf::Shape::Rectangle(WIN_WIDTH - 200, 0,
				  WIN_WIDTH - 198, WIN_HEIGHT,
				  sf::Color::Red));
    
    if(hw_on) {
      app.Draw(stop);
      col = DEST_COLOR;
      draw_point(dest);
      col = POS_COLOR;
      draw_point(pos);
      dest_str.SetText(std::string("(") + int2str(dest.x()) + ", " + int2str(dest.y()) + ")");
      app.Draw(dest_str);
      pos_str.SetText(std::string("(") + int2str(pos.x()) + ", " + int2str(pos.y()) + ")");
      app.Draw(pos_str);
    }
    else {
      app.Draw(start);
    }
    
    if(warning) {
      warning--;
      app.Draw(obs_box);
      app.Draw(obs);
    }
    
    app.Display();
      
    sf::Sleep(0.050f);
  }
  
  if(fd != -1) {
    close(fd);
  }
  return EXIT_SUCCESS;
}
