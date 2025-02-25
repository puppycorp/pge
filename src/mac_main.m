#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import "shaders.h"

@interface Renderer : NSObject <MTKViewDelegate>
@property (nonatomic, strong) id<MTLDevice> device;
@property (nonatomic, strong) id<MTLCommandQueue> commandQueue;
@property (nonatomic, strong) id<MTLRenderPipelineState> pipelineState;
@property (nonatomic, strong) id<MTLBuffer> vertexBuffer;
- (instancetype)initWithMetalKitView:(MTKView *)mtkView;
@end

@implementation Renderer

- (instancetype)initWithMetalKitView:(MTKView *)mtkView {
    if ((self = [super init])) {
        self.device = mtkView.device;
        self.commandQueue = [self.device newCommandQueue];

        static const float vertices[] = {
             0.0f,  0.5f, 0.0f, 1.0f,
            -0.5f, -0.5f, 0.0f, 1.0f,
             0.5f, -0.5f, 0.0f, 1.0f,
        };
        self.vertexBuffer = [self.device newBufferWithBytes:vertices
                                                     length:sizeof(vertices)
                                                    options:MTLResourceStorageModeShared];

        NSData *libraryData = [NSData dataWithBytes:shaders_metallib length:shaders_metallib_len];
        NSLog(@"Embedded shader data length: %lu", (unsigned long)[libraryData length]);
        if ([libraryData length] == 0) {
            NSLog(@"Embedded shader data is empty!");
            exit(-1);
        }

        // Convert NSData to dispatch_data_t
        dispatch_data_t dData = dispatch_data_create(libraryData.bytes, libraryData.length, DISPATCH_DATA_DESTRUCTOR_DEFAULT, NULL);

        NSError *error = nil;
        id<MTLLibrary> library = [self.device newLibraryWithData:dData error:&error];
        if (!library) {
            NSLog(@"Error loading embedded Metal library: %@", error);
            exit(-1);
        }

        id<MTLFunction> vertexFunction = [library newFunctionWithName:@"vertex_main"];
        id<MTLFunction> fragmentFunction = [library newFunctionWithName:@"fragment_main"];

        if (!vertexFunction || !fragmentFunction) {
            NSLog(@"Error: Could not find required shader functions.");
            exit(-1);
        }

        MTLRenderPipelineDescriptor *pipelineDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
        pipelineDescriptor.vertexFunction   = vertexFunction;
        pipelineDescriptor.fragmentFunction = fragmentFunction;
        pipelineDescriptor.colorAttachments[0].pixelFormat = mtkView.colorPixelFormat;

        self.pipelineState = [self.device newRenderPipelineStateWithDescriptor:pipelineDescriptor error:&error];
        if (!self.pipelineState) {
            NSLog(@"Error creating pipeline state: %@", error);
            exit(-1);
        }
    }
    return self;
}

- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
    // Handle size change if needed.
}

- (void)drawInMTKView:(MTKView *)view {
    id<MTLCommandBuffer> commandBuffer = [self.commandQueue commandBuffer];
    MTLRenderPassDescriptor *renderPassDescriptor = view.currentRenderPassDescriptor;

    if (renderPassDescriptor) {
        id<MTLRenderCommandEncoder> renderEncoder =
            [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
        [renderEncoder setRenderPipelineState:self.pipelineState];
        [renderEncoder setVertexBuffer:self.vertexBuffer offset:0 atIndex:0];
        [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [renderEncoder endEncoding];

        [commandBuffer presentDrawable:view.currentDrawable];
    }
    [commandBuffer commit];
}

@end

int main(int argc, const char * argv[]) {
    @autoreleasepool {
        NSApplication *app = [NSApplication sharedApplication];
        NSRect frame = NSMakeRect(0, 0, 800, 600);
        NSWindow *window = [[NSWindow alloc] initWithContentRect:frame
                                                       styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable)
                                                         backing:NSBackingStoreBuffered
                                                           defer:NO];
        [window setTitle:@"PGE - Puppy Game Engine"];

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device) {
            NSLog(@"Metal is not supported on this device");
            return -1;
        }

        MTKView *mtkView = [[MTKView alloc] initWithFrame:frame device:device];
        mtkView.clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 1.0);
        mtkView.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
        [window setContentView:mtkView];
        [window makeKeyAndOrderFront:nil];

        Renderer *renderer = [[Renderer alloc] initWithMetalKitView:mtkView];
        if (!renderer) {
            NSLog(@"Failed to initialize renderer.");
            return -1;
        }
        mtkView.delegate = renderer;

        [app run];
    }
    return 0;
}