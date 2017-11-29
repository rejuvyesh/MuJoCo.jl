# version of library to download
const version = v"1.50"

using BinDeps
@BinDeps.setup

function compatible_version(lib, handle)
   major, minor, rev = Ref{Cint}(), Ref{Cint}(), Ref{Cint}()
   f = Libdl.dlsym(handle, :mj_version)
   f == C_NULL && return false
   v = ccall(f, Int32, ())
   return v >= 150
end

baseurl = "https://www.roboti.us/download/mjpro150_"
basedir = dirname(@__FILE__)
unpack = joinpath(basedir, "mjpro150")
libpath = unpack*"/bin"

# library source code
if is_linux()
   push!(BinDeps.defaults, Binaries)

   # First find three library dependents
   mujoco_glfw= library_dependency("libglfw", aliases=["libglfw.so.3"])
   mujoco_glew= library_dependency("libglew", aliases=["libglew"])
   mujoco_GL  = library_dependency("libGL") # looks in system
   mujoco_nix = library_dependency("libmujoco", aliases=["libmujoco150.so", "libmujoco150"])

   url = baseurl*"linux.zip"
   if isdir(unpack) == false
      info("Downloading: ", url, " to ", unpack)
      file = Base.download(url) # to /tmp
      run(`unzip -o $file -d $basedir`)
   end

   preloads = string("Libdl.dlopen(\"$(libpath)/libglew.so\", Libdl.RTLD_LAZY | Libdl.RTLD_DEEPBIND | Libdl.RTLD_GLOBAL)")

   provides(Binaries, URI(url), mujoco_glfw, unpacked_dir=unpack, installed_libpath=libpath)
   provides(Binaries, URI(url), mujoco_glew, unpacked_dir=unpack, installed_libpath=libpath)
   eval(parse(preloads))
   provides(Binaries, URI(url), mujoco_nix, unpacked_dir=unpack, installed_libpath=libpath, preload=preloads)

   @BinDeps.install Dict([(:libglfw, :libglfw),
                          (:libglew, :libglew),
                          (:libGL, :libGL),
                          (:libmujoco, :libmujoco)])
elseif is_apple()
   mujoco_osx = library_dependency("libmujoco", aliases=["libmujoco150"], validate=compatible_version)
   url = baseurl*"osx.zip"
   info("Downloading: ", url, " to ", unpack)
   provides(Binaries, URI(url), mujoco_osx, unpacked_dir=unpack, installed_libpath=libpath)
   #provides(SimpleBuild,
   #         (@build_steps begin
   #             CreateDirectory(joinpath(basedir, "downloads"))
   #             FileDownloader(string(url),
   #                            joinpath(basedir, "downloads/mjpro150_osx.zip"))
   #             FileUnpacker(joinpath(basedir, "downloads/mjpro150_osx.zip"),
   #                          basedir, "mjpro150")
   #          end), mujoco_osx, installed_libpath=libpath)
   @BinDeps.install Dict(:libmujoco=>:libmujoco)
elseif is_windows()
   mujoco_win = library_dependency("libmujoco", aliases=["libmujoco150"], validate=compatible_version)
   url = baseurl*"win$(Sys.WORD_SIZE).zip"
   info("Downloading: ", url, " to ", unpack)
   provides(Binaries, URI(url), mujoco_win, unpacked_dir=unpack, installed_libpath=libpath)
   @BinDeps.install Dict(:libmujoco=>:libmujoco)
end

is_linux() && pop!(BinDeps.defaults)

