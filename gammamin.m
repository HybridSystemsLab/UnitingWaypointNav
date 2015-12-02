function gamma = gammamin(~)
gamma=fzero(@(gamma) betamin(gamma),0.1);