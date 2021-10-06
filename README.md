spacesim is a c-shared library for basic physics using rectangles and fixed point arithmetic. It is used by the go-space-serv and go-space-cli projects here.
The reason it is a c-shared library is so that both the server and client can use the exact same code to run the simulation.
Here are some short gifs to see it in action.

Here is a controlled body with a debug ghost shooting a bouncing bomb.
![](https://gyazo.com/1c2078ee82558974f26f4bb2df23998f.gif)


Here is some collision detection. It highlights that players can both bounce and slide on walls that aretile based.
![](https://gyazo.com/1bee73166583b491c015910aa0590b47.gif)
