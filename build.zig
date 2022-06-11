const std = @import("std");

pub fn build(b: *std.build.Builder) !void {
    const pico_exe = try PicoExe.init(b, "testbin", "src/main.zig");

    pico_exe.install();
}

// TODO: Right now this build file only works when ran from the zig-pico-sdk
// dir. Figure out how any other build.zig file could @import this file and use
// PicoExe to build a project.

pub const PicoExe = struct {
    const base_id = .custom;

    step: std.build.Step,
    builder: *std.build.Builder,
    name: []const u8,
    zig_obj: *std.build.LibExeObjStep,

    pio_files: std.ArrayList([]const u8),

    output_bin: std.build.GeneratedFile,

    pub fn init(builder: *std.build.Builder, name_raw: []const u8, root_src: []const u8) !*PicoExe {
        const PATH = std.fs.path.dirname(@src().file) orelse unreachable;

        // Check if the pico-sdk is cloned
        // TODO: Add support for system wide pico-sdk installation
        const build_root = std.fs.openDirAbsolute(PATH, .{}) catch unreachable;
        try build_root.access("pico-sdk/src", .{});

        const name = builder.dupe(name_raw);

        const target = std.zig.CrossTarget{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
            .os_tag = .freestanding,
            .abi = .eabi,
        };
        const mode = builder.standardReleaseOptions();

        const obj = builder.addObject(name_raw, root_src);
        obj.setTarget(target);
        obj.setBuildMode(mode);

        // TODO: Find these headers dynamically
        obj.addSystemIncludeDir("/usr/arm-none-eabi/include");

        // TODO: This include is tricky. This is generated on calling 'cmake',
        // either for the project (in zig-cache/cmake) or pico-sdk itself.
        // Preferably this will be generated in cache, but the CMakeFile.txt
        // file requires the to be linked obj file as well.
        // One solution could be to run `cmake ../../pico-sdk` from a folder in
        // zig-cache.
        const generated = builder.pathJoin(&.{PATH, "pico-sdk/build/generated/pico_base"});
        obj.addSystemIncludeDir(generated);
        //obj.addSystemIncludeDir("zig-cache/cmake/build/generated/pico_base");

        // TODO: Dynamically find this path
        // This is where the name.pio.h files pio files end up
        const cmake_build_path = builder.pathJoin(&.{builder.build_root, builder.cache_root, "cmake/build"});
        obj.addIncludeDir(cmake_build_path);

        // TODO: Maybe find these headers dynamically as well
        const sdk_includes: []const []const u8 = &.{
            "pico-sdk/src/common/boot_picoboot/include",
            "pico-sdk/src/common/boot_uf2/include",
            "pico-sdk/src/common/pico_base/include",
            "pico-sdk/src/common/pico_binary_info/include",
            "pico-sdk/src/common/pico_bit_ops/include",
            "pico-sdk/src/common/pico_divider/include",
            "pico-sdk/src/common/pico_stdlib/include",
            "pico-sdk/src/common/pico_sync/include",
            "pico-sdk/src/common/pico_time/include",
            "pico-sdk/src/common/pico_usb_reset_interface/include",
            "pico-sdk/src/common/pico_util/include",
            "pico-sdk/src/rp2040/hardware_regs/include",
            "pico-sdk/src/rp2040/hardware_structs/include",
            "pico-sdk/src/rp2_common/boot_stage2/asminclude",
            "pico-sdk/src/rp2_common/boot_stage2/include",
            "pico-sdk/src/rp2_common/cmsis/include",
            "pico-sdk/src/rp2_common/hardware_adc/include",
            "pico-sdk/src/rp2_common/hardware_base/include",
            "pico-sdk/src/rp2_common/hardware_claim/include",
            "pico-sdk/src/rp2_common/hardware_clocks/include",
            "pico-sdk/src/rp2_common/hardware_divider/include",
            "pico-sdk/src/rp2_common/hardware_dma/include",
            "pico-sdk/src/rp2_common/hardware_exception/include",
            "pico-sdk/src/rp2_common/hardware_flash/include",
            "pico-sdk/src/rp2_common/hardware_gpio/include",
            "pico-sdk/src/rp2_common/hardware_i2c/include",
            "pico-sdk/src/rp2_common/hardware_interp/include",
            "pico-sdk/src/rp2_common/hardware_irq/include",
            "pico-sdk/src/rp2_common/hardware_pio/include",
            "pico-sdk/src/rp2_common/hardware_pll/include",
            "pico-sdk/src/rp2_common/hardware_pwm/include",
            "pico-sdk/src/rp2_common/hardware_resets/include",
            "pico-sdk/src/rp2_common/hardware_rtc/include",
            "pico-sdk/src/rp2_common/hardware_spi/include",
            "pico-sdk/src/rp2_common/hardware_sync/include",
            "pico-sdk/src/rp2_common/hardware_timer/include",
            "pico-sdk/src/rp2_common/hardware_uart/include",
            "pico-sdk/src/rp2_common/hardware_vreg/include",
            "pico-sdk/src/rp2_common/hardware_watchdog/include",
            "pico-sdk/src/rp2_common/hardware_xosc/include",
            "pico-sdk/src/rp2_common/pico_bootrom/include",
            "pico-sdk/src/rp2_common/pico_double/include",
            "pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include",
            "pico-sdk/src/rp2_common/pico_float/include",
            "pico-sdk/src/rp2_common/pico_int64_ops/include",
            "pico-sdk/src/rp2_common/pico_malloc/include",
            "pico-sdk/src/rp2_common/pico_mem_ops/include",
            "pico-sdk/src/rp2_common/pico_multicore/include",
            "pico-sdk/src/rp2_common/pico_platform/include",
            "pico-sdk/src/rp2_common/pico_printf/include",
            "pico-sdk/src/rp2_common/pico_runtime/include",
            "pico-sdk/src/rp2_common/pico_stdio/include",
            "pico-sdk/src/rp2_common/pico_stdio_semihosting/include",
            "pico-sdk/src/rp2_common/pico_stdio_uart/include",
            "pico-sdk/src/rp2_common/pico_stdio_usb/include",
            "pico-sdk/src/rp2_common/pico_unique_id/include",
        };

        for (sdk_includes) |inc| {
            const path = builder.pathJoin(&.{PATH, inc});
            obj.addIncludeDir(path);
        }

        //src/boards/include/
        //obj.addIncludeDir("pico-sdk/src/common/boot_picoboot/include");
        //obj.addIncludeDir("pico-sdk/src/common/boot_uf2/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_base/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_binary_info/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_bit_ops/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_divider/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_stdlib/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_sync/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_time/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_usb_reset_interface/include");
        //obj.addIncludeDir("pico-sdk/src/common/pico_util/include");
        //obj.addIncludeDir("pico-sdk/src/rp2040/hardware_regs/include");
        //obj.addIncludeDir("pico-sdk/src/rp2040/hardware_structs/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/boot_stage2/asminclude");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/boot_stage2/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/cmsis/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_adc/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_base/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_claim/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_clocks/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_divider/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_dma/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_exception/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_flash/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_gpio/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_i2c/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_interp/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_irq/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_pio/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_pll/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_pwm/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_resets/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_rtc/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_spi/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_sync/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_timer/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_uart/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_vreg/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_watchdog/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/hardware_xosc/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_bootrom/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_double/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_float/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_int64_ops/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_malloc/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_mem_ops/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_multicore/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_platform/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_printf/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_runtime/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_stdio/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_stdio_semihosting/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_stdio_uart/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_stdio_usb/include");
        //obj.addIncludeDir("pico-sdk/src/rp2_common/pico_unique_id/include");

        const self = builder.allocator.create(PicoExe) catch unreachable;
        self.* = PicoExe {
            .builder = builder,
            .step = std.build.Step.init(base_id, name, builder.allocator, make),
            .name = name,
            .zig_obj = obj,
            .output_bin = .{ .step = &self.step },

            .pio_files = std.ArrayList([]const u8).init(builder.allocator),
        };

        self.step.dependOn(&obj.step);

        return self;
    }

    pub fn addPioSourceFile(self: *PicoExe, file: []const u8) void {
        self.pio_files.append(file) catch unreachable;
    }

    pub fn install(self: *PicoExe) void {
        const bin_install_step = self.builder.addInstallBinFile(
            std.build.FileSource{ .generated = &self.output_bin }, self.name
        );
        bin_install_step.step.dependOn(&self.step);

        self.builder.getInstallStep().dependOn(&bin_install_step.step);
    }

    pub fn make(step: *std.build.Step) !void {
        const self = @fieldParentPtr(PicoExe, "step", step);
        const builder = self.builder;

        const log = std.log.scoped(.PicoExe);

        const zig_obj_path = self.zig_obj.getOutputSource().getPath(builder);

        log.info("Creating build directory and CMakeLists file for the pico-sdk", .{});

        const build_root = try std.fs.openDirAbsolute(builder.build_root, .{});
        const cache_root = try build_root.openDir(builder.cache_root, .{});

        // Create build directory in cache, and CMakeLists.txt file
        cache_root.makePath("cmake/build") catch {};
        const cmake_dir = try cache_root.openDir("cmake", .{});
        const cmake_cfg = try cmake_dir.createFile("CMakeLists.txt", .{});
        defer cmake_cfg.close();

        const PATH = std.fs.path.dirname(@src().file) orelse unreachable;

        // Fill in CMakeLists.txt file used by pico-sdk
        const writer = cmake_cfg.writer();
        try writer.print(
            \\cmake_minimum_required(VERSION 3.13)
            \\include({s}/pico-sdk/pico_sdk_init.cmake)
            \\project({s})
            \\pico_sdk_init()
            \\add_executable({s})
            \\target_link_libraries({s} pico_stdlib {s})
            \\#pico_add_extra_outputs({s})
            \\
            , .{PATH, self.name, self.name, self.name, zig_obj_path, self.name}
        );

        // Add PIO files to CMakelists.txt
        for (self.pio_files.items) |pio_file| {
            const pio_path = builder.pathJoin(&.{builder.build_root, pio_file});
            try writer.print(
                "pico_generate_pio_header({s} {s})\n",
                .{self.name, pio_path});
        }

        var args = std.ArrayList([]const u8).init(builder.allocator);
        defer args.deinit();

        const cmake_path = builder.pathJoin(&.{builder.build_root, builder.cache_root, "cmake"});
        const build_path = builder.pathJoin(&.{cmake_path, "build"});

        // Running cmake command
        args.append("cmake") catch unreachable;
        args.append("-S") catch unreachable;
        args.append(cmake_path) catch unreachable;
        args.append("-B") catch unreachable;
        args.append(build_path) catch unreachable;

        log.info("Running cmake", .{});
        const cmake_out = try builder.execFromStep(args.items, &self.step);
        log.info("cmake outptut:\n{s}", .{cmake_out});

        // Running make command
        args.clearRetainingCapacity();
        args.append("make") catch unreachable;
        args.append("-C") catch unreachable;
        args.append(build_path) catch unreachable;

        log.info("Running make", .{});
        const make_out = try builder.execFromStep(args.items, &self.step);
        log.info("make output:\n{s}", .{make_out});

        // Add the generated elf file to the 'output_bin' GeneratedFile path
        const out_file = try std.fmt.allocPrint(builder.allocator, "{s}.elf", .{self.name});
        self.output_bin.path = builder.pathJoin(
            &.{ build_path, out_file }
        );

        log.info("Done!", .{});
    }
};
