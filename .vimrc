set makeprg=[[\ -f\ Makefile\ ]]\ &&\ make\ \\\|\\\|\ make\ -C\ build/

set grepprg=grep\ -nrI\ --exclude-dir={.git,build}\ --exclude=*tags

" recreate tags file with F5
map <F5> :GenerateInfiniTAMTags<CR>

set tags+=infinitam.tags

function GenerateInfiniTAMTags()
  !ctags -R --sort=yes --langmap=C++:+.cu --c++-kinds=+p --fields=+iaS --extra=+qf --exclude=build --exclude=.git --exclude=cmake -f ./infinitam.tags .
endfunction
command GenerateInfiniTAMTags execute GenerateInfiniTAMTags()
