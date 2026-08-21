function wp_gen()

X = randi([-100,100],10,1);
Y = randi([-100,100],10,1);

wp_list = [X Y];

assignin('base','wp_list',wp_list);

end